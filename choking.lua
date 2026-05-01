local M = {}

local min = math.min
local max = math.max

-- Utility function to safely access values
local function safeGet(t, k, default)
    if t == nil then return default end
    return t[k] or default
end

-- Main update function for the choking system
local function updateChoking(device, dt, electrics)
    -- Initialize device properties if they don't exist
    device.hydrolockTimer = device.hydrolockTimer or 0
    device.prevThrottlePos = device.prevThrottlePos or 0
    device.chokeFloodAccumulator = device.chokeFloodAccumulator or 0

    -- Safely get electrics values
    local electricsValues = safeGet(electrics, 'values', {})
    local throttlePos = safeGet(electricsValues, device.electricsThrottleName, 0)

    -- Calculate throttle delta
    local throttleDelta = throttlePos - (device.prevThrottlePos or 0)
    device.prevThrottlePos = throttlePos

    -- Choke-based flood contribution:
    -- Over-choking while cranking increases flooding tendency
    local chokeEffect = device.chokeEffect or 0
    local isCranking = device.starterEngagedCoef and device.starterEngagedCoef > 0
    if isCranking and chokeEffect > 0.5 then
        -- Excess choke during cranking gradually accumulates flood contribution
        local excessChoke = chokeEffect - 0.5
        device.chokeFloodAccumulator = min(1.0, device.chokeFloodAccumulator + excessChoke * 0.01 * dt)
    else
        -- Recover when not cranking or choke is low
        device.chokeFloodAccumulator = max(0, device.chokeFloodAccumulator - 0.02 * dt)
    end

    -- Handle starter disengage timer
    if device.timeSinceStarterDisengage ~= nil then
        device.timeSinceStarterDisengage = (device.timeSinceStarterDisengage or 0) + dt
        if device.timeSinceStarterDisengage > 2 then
            device.timeSinceStarterDisengage = nil
        end
    end

    -- Update hydrolock timer
    device.hydrolockTimer = (device.hydrolockTimer or 0) + dt

    return {
        throttleDelta = throttleDelta
    }
end

-- Returns a per-tick flood contribution delta for the per-cylinder system.
-- This replaces the old device.floodLevel = 0 override.
local function getFloodContribution(self, device, dt)
    local contribution = 0

    -- Choke-related flooding
    local accumulator = device.chokeFloodAccumulator or 0
    if accumulator > 0.1 then
        contribution = contribution + accumulator * 0.003 * dt
    end

    -- Hydrolock contribution (water ingestion via intake when submerged)
    -- Only applies when actually submerged (isHydrolocking set by combustionEngine's checkHydroLocking)
    if device.isHydrolocking and device.canFlood then
        contribution = contribution + 0.01 * dt
    end

    return contribution
end

-- Battery update function (kept for compatibility)
local function updateFloodHydrolockBattery(device, dt)
    -- Initialize battery if not exists
    if not device.starterBattery then
        device.starterBattery = {
            storedEnergy = 100000,
            maxEnergy = 100000
        }
    end

    -- Update battery state
    if device.starterEngagedCoef and device.starterEngagedCoef > 0 then
        local drainRate = 2
        device.starterBattery.storedEnergy = max(0,
            device.starterBattery.storedEnergy - (drainRate * dt))
    end

    return device.starterBattery
end

-- Module exports
M.update = updateChoking
M.getFloodContribution = getFloodContribution
M.updateFloodHydrolockBattery = updateFloodHydrolockBattery

return M
