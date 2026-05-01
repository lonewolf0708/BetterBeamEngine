-- Carburetor simulation for BeamNG.drive
-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local carburetor = {}
local carburetor_mt = { __index = carburetor }

local max = math.max
local min = math.min
local abs = math.abs
local random = math.random

local rpmToAV = 0.104719755
local avToRPM = 9.549296596425384
local torqueToPower = 0.0003404345295653085
local psToWatt = 735.499
local hydrolockThreshold = 0.9
-- Temperature conversion functions with nil check
local function kelvinToFahrenheit(kelvin)
    if not kelvin or type(kelvin) ~= 'number' then
        return 70  -- Default to room temperature if invalid
    end
    return (kelvin - 273.15) * 9/5 + 32
end

-- Constants for carburetor simulation
local CARBURETOR_CONSTANTS = {
    -- Fuel injection constants
    baseFuelAmount = 0.05,        -- Base fuel amount per cycle
    maxFuelPerCylinder = 1.0,     -- Maximum fuel per cylinder
    minFuelForInjection = 0.2,    -- Minimum fuel for injection
    minFuelForCombustion = 0.15,  -- Minimum fuel needed for combustion
    minAirForCombustion = 0.4,    -- Minimum air needed for combustion (when warm)
    crankingFuelMultiplier = 1.5, -- Fuel multiplier during cranking
    
    -- Temperature adjustment factors
    minTempForCombustion = -30,   -- °C - minimum temperature where combustion is possible
    maxTempForCombustion = 150,   -- °C - maximum temperature before fuel vaporization issues
    tempAdjustmentRange = 100,    -- °C - range over which temperature affects combustion
    
    -- Base enrichment values
    coldEnrichmentMax = 3.0,      -- Maximum cold enrichment multiplier
    warmEnrichment = 1.2,         -- Warm-up enrichment multiplier
    hotEnrichment = 0.9,          -- Hot enrichment multiplier (may be <1 for rich/lean adjustment)
    
    -- Float bowl levels
    -- Float bowl levels
    maxFloatLevel = 1.0,  -- meters (default)
    minFloatLevel = 0.1,  -- meters (default)
    
    -- Float bowl properties (diesel-specific)
    fuelBowlVolume = 0.001,  -- 1 liter
    fuelDensity = 850,      -- kg/m³ (diesel)
    
    -- Temperature conversion constants
    kelvinToCelsius = -273.15,
    celsiusToFahrenheit = 9/5,
    celsiusToFahrenheitOffset = 32,
    
    -- Temperature thresholds (°F)
    veryColdTempThreshold = 0,     -- 0°F (-17.8°C) - Cold start
    coldTempThreshold = 32,       -- 32°F (0°C) - Warm up start
    warmTempThreshold = 68,       -- 68°F (20°C) - Operating temp
    hotTempThreshold = 194,       -- 194°F (90°C) - Hot operating temp
    
    -- Idle multipliers (added to base value)
    veryColdIdleMultiplier = 0.2,  -- Base 600 RPM * 0.2 + 200 = ~820 RPM
    coldIdleMultiplier = 0.15,     -- Base 600 RPM * 0.15 + 150 = ~740 RPM
    warmIdleMultiplier = 0.05,     -- Base 600 RPM * 0.05 + 50 = ~630 RPM
    hotIdleMultiplier = 0.01,      -- Base 600 RPM * 0.01 + 30 = ~36 RPM
    
    -- Cranking combustion settings
    crankingCombustionThreshold = 0.05,  -- 5% float level
    crankingCombustionSoundVolume = 0.9,  -- Sound volume multiplier
    crankingCombustionChance = 0.10,  -- Chance per second of a combustion event during cranking (10%)
    crankingCombustionBackfireChance = 0.05,  -- Chance per second of a backfire event during cranking (5%)
    crankingCombustionDuration = 0.1, -- Duration in seconds of a cranking combustion event
    crankingCombustionMinTemp = 0,   -- Minimum temperature (°F) for cranking combustion
    crankingCombustionMaxTemp = 200, -- Maximum temperature (°F) for cranking combustion
    
    -- Fuel enrichment factors (diesel-specific)
    coldStartEnrichment = 1.5,    -- More enrichment for cold start
    warmUpEnrichment = 1.2,       -- Richer mix during warm up
    
    -- Choke settings (diesel-specific)
    chokeIdleMultiplier = 1.2,    -- Base 550 RPM + 110 = ~660 RPM
    chokePullOffTime = 30.0,      -- seconds
    chokePullOffTimer = 15,
    
    -- Disable choke when engine is warm/hot
    disableChokeWhenWarm = true,  -- Disable choke when engine is warm/hot
    warmThreshold = 68,           -- Temperature threshold in °F for disabling choke (20°C)
    hotThreshold = 194,           -- Temperature threshold in °F for disabling choke (90°C)
    
    -- Roughness settings (diesel-specific)
    idleRoughnessCold = 2000,     -- Very rough when cold
    idleRoughnessWarm = 600,      -- Smooth when warm
    idleRoughnessSmoothingRate = 0.03,  -- Smoothing rate per second
    
    -- Accelerator pump settings
    maxPumpFuel = 0.3,            -- Maximum amount of fuel the accelerator pump can add
    pumpDecayRate = 0.5,          -- Rate at which the pump effect decays (per second)
    
    -- Fuel system settings
    venturiSize = 0.02,           -- meters
    fuelVaporizationTemp = 140,   -- Higher vaporization temp for diesel
    fuelVaporizationRate = 0.003,  -- Slower vaporization rate
    coldStartFuelRate = 0.003,     -- Higher fuel rate for cold starts
    warmUpFuelRate = 0.002,       -- Higher fuel rate during warm up
    
    -- Manual choke control
    manualChokeStep = 0.1,        -- Step size for manual choke control
    maxManualChoke = 1.0,         -- Maximum manual choke position
    minManualChoke = 0.0,         -- Minimum manual choke position
    
    -- Fuel mixture settings
    baseFuelMixture = 1.0,        -- Base fuel mixture ratio
    maxFuelMixture = 1.5,         -- Maximum fuel mixture ratio
    minFuelMixture = 0.5,         -- Minimum fuel mixture ratio
    
    -- Idle valve settings
    idleValveMaxPosition = 1.0,   -- Maximum idle valve position
    idleValveMinPosition = 0.0,   -- Minimum idle valve position
    idleValveStep = 0.1,          -- Step size for idle valve control
    
    -- Starvation settings
    starvationThreshold = 0.1,    -- 10% float level
    starvationMultiplier = 0.5,
    
    -- Fuel enrichment settings
    coldEnrichmentMultiplier = 1.5,
    warmEnrichmentMultiplier = 1.2,
    hotEnrichmentMultiplier = 0.8,
    
    -- Surge settings
    surgeThreshold = 0.3,       -- throttle change threshold
    surgeDuration = 0.25,        -- seconds
    surgeTorqueMultiplier = 1.2,  -- Torque boost during surge
    surgeFuelMultiplier = 1.5,  -- Fuel enrichment during surge
    surgeSoundVolume = 0.9,  -- Sound volume multiplier
    
    -- Diesel-specific settings
    dieselFuelVaporizationTemp = 100,    -- Higher vaporization temp for diesel
    dieselFuelVaporizationRate = 0.005,  -- Slower vaporization rate
    dieselColdStartFuelRate = 0.002,     -- Higher fuel rate for cold starts
    dieselWarmUpFuelRate = 0.0015,       -- Higher fuel rate during warm up
    dieselIdleRoughnessCold = 1500,      -- Very rough when cold
    dieselIdleRoughnessWarm = 500,       -- Smooth when warm
    dieselIdleRoughnessSmoothingRate = 0.02,  -- Smoothing rate per second
    
    -- Oil-bath air filter settings
    oilBathFilterEfficiency = 0.95,      -- Filter efficiency
    oilBathFilterDirtThreshold = 0.7,    -- Dirt accumulation threshold
    oilBathFilterDirtRecoveryRate = 0.01,  -- Recovery rate per second
    oilBathFilterSoundVolume = 0.8,      -- Sound volume multiplier
    
    -- Multifuel settings
    multifuelKeroseneEfficiency = 0.85,   -- Kerosene efficiency
    multifuelGasolineEfficiency = 0.75,   -- Gasoline efficiency
    multifuelMixRate = 0.02,             -- Mix rate per second
    multifuelMinMix = 0.1,               -- Minimum mix ratio
    multifuelMaxMix = 0.9,               -- Maximum mix ratio

    -- Missing integration constants
    maxRefillRate = 0.5,                 -- Max fuel bowl refill rate
    chokeEffectCold = 0.8,               -- Choke effect when very cold
    chokeEffectWarm = 0.3,               -- Choke effect when warming up
    chokeThrottleBoostCold = 0.05,       -- Throttle boost when very cold
    chokeThrottleBoostWarm = 0.02,       -- Throttle boost when warming up
}

-- Add constants to carburetor module
carburetor.constants = CARBURETOR_CONSTANTS

-- Initialize carburetor state
function carburetor:new(device, floatLevel, chokeActive)
    -- Store device reference first
    local carb = setmetatable({}, carburetor_mt)
    carb.device = device
    
    -- Initialize basic states
    carb.floatLevel = floatLevel or CARBURETOR_CONSTANTS.maxFloatLevel * 0.8
    carb.chokeActive = chokeActive or false
    carb.chokePullOffTimer = 0
    carb.chokePullOffActive = false
    carb.fuelEnrichment = 1.0
    carb.vaporizationLevel = 0
    carb.starvationLevel = 0
    carb.isStarving = false
    carb.lastRefillTime = 0
    
    -- Initialize fuel bowl properties
    carb.fuelBowlVolume = 0.001  -- 1 liter (default)
    carb.fuelDensity = 750  -- kg/m³ (default)
    
    -- Calculate initial fuel mass
    carb.fuelMass = carb.floatLevel * carb.fuelBowlVolume * carb.fuelDensity
    
    -- Initialize diesel-specific states
    carb.currentFuelMix = 1.0  -- 1.0 = 100% diesel, 0.0 = 100% alternative fuel
    carb.oilBathFilterDirt = 0
    carb.oilBathFilterSound = nil
    
    -- Initialize diesel-specific states from device params
    if device and device.params then
        carb.isDiesel = device.params.isDiesel
        carb.isMultifuel = device.params.isMultifuel
    else
        carb.isDiesel = false
        carb.isMultifuel = false
    end
    
    -- Initialize other states
    carb.isCrankingCombustion = false
    carb.crankingCombustionTimer = 0
    carb.crankingCombustionSound = nil
    carb.isSurging = false
    carb.surgeTimer = 0
    carb.lastThrottle = 0
    carb.surgeSound = nil
    
    -- Initialize idle valve and fuel mixture
    carb.idleValvePosition = CARBURETOR_CONSTANTS.idleValveMinPosition
    carb.fuelMixtureRatio = CARBURETOR_CONSTANTS.baseFuelMixture
    
    -- Initialize accelerator pump state
    carb.acceleratorPumpFuel = 0
    carb.pumpSoundPlayed = false
    
    return carb
end

-- Initialize carburetor parameters and bindings
function carburetor:initialize()
    if self.device then
        -- Get engine-specific carburetor parameters
        self.params = self.device.carburetorParams or {}
        
        -- Update float bowl properties if engine-specific values are provided
        if self.params.fuelBowlVolume then
            self.fuelBowlVolume = self.params.fuelBowlVolume
            self.fuelMass = self.floatLevel * self.fuelBowlVolume * self.fuelDensity
        end
        
        if self.params.fuelDensity then
            self.fuelDensity = self.params.fuelDensity
            self.fuelMass = self.floatLevel * self.fuelBowlVolume * self.fuelDensity
        end
        
        -- Add input bindings for choke control
        if self.device.addInputBinding then
            self.device:addInputBinding("chokePull", "keyPress", "T", function() self:setChoke(true) end)
            self.device:addInputBinding("chokeRelease", "keyPress", "G", function() self:setChoke(false) end)
            
            -- Add input bindings for multifuel control if applicable
            if self.isMultifuel then
                self.device:addInputBinding("increaseKerosene", "keyPress", "I", function() self:increaseKerosene() end)
                self.device:addInputBinding("increaseGasoline", "keyPress", "K", function() self:increaseGasoline() end)
            end
        end
    end
end

-- Update oil-bath air filter state
function carburetor:updateOilBathFilter(dt)
    if not self.isDiesel then return end
    
    -- Simulate dirt accumulation based on operating conditions
    local temp = self.device.thermals.engineBlockTemperature
    local rpm = self.device.outputAV1 * 9.549296596425384
    local throttle = self.device.throttle or 0
    
    -- Higher RPM and throttle increase dirt accumulation
    local dirtAccumulation = (rpm * throttle * 0.0001) * dt
    
    -- Dirt recovery when engine is off or at idle
    if rpm < 100 then
        dirtAccumulation = -CARBURETOR_CONSTANTS.oilBathFilterDirtRecoveryRate * dt
    end
    
    self.oilBathFilterDirt = max(0, min(1, self.oilBathFilterDirt + dirtAccumulation))
    
    -- Play filter sound when dirty
    if self.oilBathFilterDirt > CARBURETOR_CONSTANTS.oilBathFilterDirtThreshold and not self.oilBathFilterSound then
        self.oilBathFilterSound = obj:createSFXSource2("event:>Engine>OilBathFilter", "AudioDefaultLoop3D", "", self.device.engineNodeID, 0)
        self.oilBathFilterSound:setVolume(CARBURETOR_CONSTANTS.oilBathFilterSoundVolume)
        self.oilBathFilterSound:play()
    elseif self.oilBathFilterDirt <= CARBURETOR_CONSTANTS.oilBathFilterDirtThreshold and self.oilBathFilterSound then
        self.oilBathFilterSound:stop()
        self.oilBathFilterSound = nil
    end
end

-- Update multifuel mix state
function carburetor:updateMultifuelMix(dt)
    if not self.isMultifuel then return end
    
    -- Simulate fuel mixture evaporation
    if self.currentFuelMix < 1.0 then
        self.currentFuelMix = min(1.0, self.currentFuelMix + CARBURETOR_CONSTANTS.multifuelMixRate * dt)
    end
end

-- Increase kerosene mix
function carburetor:increaseKerosene()
    if not self.isMultifuel then return end
    
    self.currentFuelMix = min(1.0, self.currentFuelMix + CARBURETOR_CONSTANTS.multifuelMixRate)
end

-- Adjust idle valve position
function carburetor:adjustIdleValve(direction)
    if not self.isDiesel then return end
    
    local step = CARBURETOR_CONSTANTS.idleValveStep * direction
    self.idleValvePosition = min(CARBURETOR_CONSTANTS.idleValveMaxPosition,
                                max(CARBURETOR_CONSTANTS.idleValveMinPosition,
                                self.idleValvePosition + step))
end

-- Clean oil bath air filter
function carburetor:cleanOilBathFilter()
    if not self.isDiesel then return end
    
    self.oilBathFilterDirt = 0
    if self.oilBathFilterSound then
        self.oilBathFilterSound:stop()
        self.oilBathFilterSound = nil
    end
end

-- Adjust fuel mixture ratio
function carburetor:adjustFuelMixture(direction)
    if not self.isDiesel then return end
    
    local step = CARBURETOR_CONSTANTS.multifuelMixRate * direction
    self.fuelMixtureRatio = min(CARBURETOR_CONSTANTS.maxFuelMixture,
                               max(CARBURETOR_CONSTANTS.minFuelMixture,
                               self.fuelMixtureRatio + step))
end

-- Increase gasoline mix
function carburetor:increaseGasoline()
    if not self.isMultifuel then return end
    
    self.currentFuelMix = max(0.0, self.currentFuelMix - CARBURETOR_CONSTANTS.multifuelMixRate)
end

-- Handle input controls for manual choke
function carburetor:handleInputs(inputs)
    -- Handle manual choke control
    if inputs and inputs.T then
        -- Increase choke position
        self.manualChokePosition = math.min(CARBURETOR_CONSTANTS.maxManualChoke, (self.manualChokePosition or 0) + CARBURETOR_CONSTANTS.manualChokeStep)
    elseif inputs and inputs.G then
        -- Decrease choke position
        self.manualChokePosition = math.max(CARBURETOR_CONSTANTS.minManualChoke, (self.manualChokePosition or 0) - CARBURETOR_CONSTANTS.manualChokeStep)
    end
end

-- Get temperature-based idle multiplier
function carburetor:getTemperatureMultiplier(temp)
    -- Use default values if params are not initialized yet
    local params = self.params or CARBURETOR_CONSTANTS
    
    -- Get temperature thresholds with fallback to defaults
    local veryColdTemp = params.veryColdTempThreshold or CARBURETOR_CONSTANTS.veryColdTempThreshold
    local coldTemp = params.coldTempThreshold or CARBURETOR_CONSTANTS.coldTempThreshold
    local warmTemp = params.warmTempThreshold or CARBURETOR_CONSTANTS.warmTempThreshold
    local hotTemp = params.hotTempThreshold or CARBURETOR_CONSTANTS.hotTempThreshold
    
    -- Get idle multipliers with fallback to defaults
    local veryColdMultiplier = params.veryColdIdleMultiplier or CARBURETOR_CONSTANTS.veryColdIdleMultiplier
    local coldMultiplier = params.coldIdleMultiplier or CARBURETOR_CONSTANTS.coldIdleMultiplier
    local warmMultiplier = params.warmIdleMultiplier or CARBURETOR_CONSTANTS.warmIdleMultiplier
    local hotMultiplier = params.hotIdleMultiplier or CARBURETOR_CONSTANTS.hotIdleMultiplier
    
    -- Apply temperature-based idle multiplier
    if temp < veryColdTemp then
        return veryColdMultiplier
    elseif temp < coldTemp then
        local t = (temp - veryColdTemp) / (coldTemp - veryColdTemp)
        return veryColdMultiplier + t * (coldMultiplier - veryColdMultiplier)
    elseif temp < warmTemp then
        local t = (temp - coldTemp) / (warmTemp - coldTemp)
        return coldMultiplier + t * (warmMultiplier - coldMultiplier)
    elseif temp < hotTemp then
        local t = (temp - warmTemp) / (hotTemp - warmTemp)
        return warmMultiplier + t * (hotMultiplier - warmMultiplier)
    else
        return hotMultiplier
    end
end

-- Reset carburetor state
function carburetor:reset(floatLevel, chokeActive)
    -- Reset float bowl
    self.floatLevel = floatLevel or CARBURETOR_CONSTANTS.maxFloatLevel * 0.8
    self.fuelMass = self.floatLevel * self.fuelBowlVolume * self.fuelDensity
    
    -- Reset choke state
    self.chokeActive = chokeActive or false
    self.chokePullOffTimer = 0
    self.chokePullOffActive = false
    
    -- Reset idle valve
    self.idleValvePosition = CARBURETOR_CONSTANTS.idleValveMinPosition
    
    -- Reset fuel mixture
    self.currentFuelMix = CARBURETOR_CONSTANTS.baseFuelMixture
    
    -- Reset oil-bath air filter state if diesel
    if self.isDiesel then
        self.oilBathFilterDirt = 0
        if self.oilBathFilterSound then
            self.oilBathFilterSound:stop()
        end
        self.oilBathFilterSound = nil
    end
    
    -- Reset multifuel state if applicable
    if self.isMultifuel then
        self.currentFuelMix = CARBURETOR_CONSTANTS.baseFuelMixture
    end

    self.isSurging = false
    self.surgeTimer = 0
    if self.surgeSound then
        obj:cutSFX(self.surgeSound)
        self.surgeSound = nil
    end
end

-- Update carburetor state
function carburetor:onPostUpdate(params)
    local carb = self
    local dt = params.dt
    local engineAV = params.engineAV
    local throttle = params.throttle
    local engineTempC = params.engineTempC
    local isCranking = params.isCranking
    local isRunning = params.isRunning
    
    -- Check if device is properly initialized
    if not carb.device then
        return
    end
    
    -- Update choke state from device input if available
    if carb.device.chokeInput ~= nil then
        self:setChoke(carb.device.chokeInput > 0)
    end
    
    -- Get current throttle position from params
    local currentThrottle = throttle or 0
    
    -- Use temperature from params (Celsius)
    local tempC = engineTempC or 0
    local temp = kelvinToFahrenheit(tempC + 273.15) -- Convert Celsius to Fahrenheit for internal logic
    
    -- Check for accelerator pump effect (throttle pump while engine off)
    if carb.device.starterEngagedCoef ~= 1 and (not carb.device.outputAV1 or math.abs(carb.device.outputAV1) < 1.0) then
        -- Detect throttle pump (quick increase in throttle position)
        local throttleDelta = currentThrottle - (carb.lastThrottle or 0)
        
        -- If throttle is being pumped (quickly pressed down), add a shot of fuel
        if throttleDelta > 0.3 and dt > 0 then
            -- Add a small amount of fuel to the intake
            carb.acceleratorPumpFuel = (carb.acceleratorPumpFuel or 0) + (0.1 * throttleDelta)
            
            -- Play a subtle fuel squirt sound if available
            if not carb.pumpSoundPlayed and obj and obj.createSFXSource and obj.playSFX then
                local sound = obj:createSFXSource("event:>Fuel>Pump>Squirt", "AudioDefaultLoop3D", "", 
                                                 carb.device.engineNodeID or 0, 0)
                if sound then
                    obj:setSFXVolume(sound, 0.3)  -- Lower volume for subtlety
                    obj:playSFX(sound)
                    -- Don't need to store the sound as it's a one-shot
                end
                carb.pumpSoundPlayed = true
            end
        else
            carb.pumpSoundPlayed = false
        end
        
        -- Gradually reduce the accelerator pump fuel effect
        if carb.acceleratorPumpFuel and carb.acceleratorPumpFuel > 0 then
            carb.acceleratorPumpFuel = math.max(0, carb.acceleratorPumpFuel - (dt * 0.5))  -- Fuel effect fades over 2 seconds
        end
    else
        -- Reset pump effect when engine is running
        carb.acceleratorPumpFuel = 0
        carb.pumpSoundPlayed = false
    end
    
    -- Check if starter is engaged
    if carb.device.starterEngagedCoef == 1 then
        -- Check for momentary combustion during cranking
        if random() < CARBURETOR_CONSTANTS.crankingCombustionChance * dt then
            carb.isCrankingCombustion = true
            carb.crankingCombustionTimer = CARBURETOR_CONSTANTS.crankingCombustionDuration
            
            -- Play cranking combustion sound
            if carb.crankingCombustionSound and obj and obj.cutSFX then
                obj:cutSFX(carb.crankingCombustionSound)
            end
            
            -- Only create sound if engine node ID is valid and sound functions are available
            if self.device and self.device.engineNodeID and obj.createSFXSource and obj.setSFXVolume and obj.playSFX then
                carb.crankingCombustionSound = obj:createSFXSource("event:>Engine>Start>Combustion", "AudioDefaultLoop3D", "", self.device.engineNodeID, 0)
                if carb.crankingCombustionSound then
                    obj:setSFXVolume(carb.crankingCombustionSound, CARBURETOR_CONSTANTS.crankingCombustionSoundVolume)
                    obj:playSFX(carb.crankingCombustionSound)
                end
            end
        end
        
        -- Check for backfire
        if random() < CARBURETOR_CONSTANTS.crankingCombustionBackfireChance * dt and 
           self.device.playAfterFireSound and 
           type(self.device.playAfterFireSound) == 'function' then
            self.device:playAfterFireSound()
        end
        
        -- Update cranking combustion timer
        if carb.isCrankingCombustion then
            carb.crankingCombustionTimer = carb.crankingCombustionTimer - dt
            if carb.crankingCombustionTimer <= 0 then
                carb.isCrankingCombustion = false
                if carb.crankingCombustionSound then
                    obj:cutSFX(carb.crankingCombustionSound)
                    carb.crankingCombustionSound = nil
                end
            end
        end
    else
        carb.isCrankingCombustion = false
        if carb.crankingCombustionSound then
            obj:cutSFX(carb.crankingCombustionSound)
            carb.crankingCombustionSound = nil
        end
    end
    
    -- Update float bowl level with safe fuel usage calculation
    local fuelUsage = (self.device.fuelUsage or 0) * dt
    carb.fuelMass = math.max(0, carb.fuelMass - fuelUsage)  -- Ensure fuel mass doesn't go negative
    carb.floatLevel = carb.fuelMass / (carb.fuelBowlVolume * carb.fuelDensity)
    
    -- Refill fuel bowl if needed
    if carb.floatLevel < CARBURETOR_CONSTANTS.minFloatLevel then
        local refillAmount = min(CARBURETOR_CONSTANTS.maxRefillRate * dt,
                                CARBURETOR_CONSTANTS.maxFloatLevel - carb.floatLevel)
        carb.floatLevel = carb.floatLevel + refillAmount
        carb.fuelMass = carb.floatLevel * carb.fuelBowlVolume * carb.fuelDensity
    end

    -- Update flooding logic
    self.floodingLevel = self.floodingLevel or 0
    if isCranking then
        -- Increase flooding if throttle is high during cranking
        local floodRate = (currentThrottle > 0.5) and 0.05 or 0.01
        self.floodingLevel = min(1.0, self.floodingLevel + floodRate * dt)
    elseif isRunning then
        -- Decrease flooding when engine is running
        self.floodingLevel = max(0, self.floodingLevel - 0.2 * dt)
    end

    -- Handle choke behavior based on temperature from params
    local temp = kelvinToFahrenheit((engineTempC or 0) + 273.15)
    
    -- Disable choke when engine is warm/hot
    if CARBURETOR_CONSTANTS.disableChokeWhenWarm and temp >= CARBURETOR_CONSTANTS.warmThreshold then
        carb.chokeActive = false
        carb.chokePullOffTimer = 0
        carb.chokePullOffActive = false
    else
        -- Handle normal choke behavior
        if carb.chokeActive then
            carb.chokePullOffTimer = min(carb.chokePullOffTimer + dt, CARBURETOR_CONSTANTS.chokePullOffTime)
            if carb.chokePullOffTimer >= CARBURETOR_CONSTANTS.chokePullOffTime then
                carb.chokePullOffActive = true
            end
        else
            carb.chokePullOffTimer = 0
            carb.chokePullOffActive = false
        end
    end
    
    -- Calculate fuel enrichment and idle multiplier based on temperature
    if temp < CARBURETOR_CONSTANTS.veryColdTempThreshold then
        carb.fuelEnrichment = CARBURETOR_CONSTANTS.coldStartEnrichment
        carb.idleMultiplier = CARBURETOR_CONSTANTS.veryColdIdleMultiplier
        carb.idleOffset = 200  -- RPM offset for very cold
    elseif temp < CARBURETOR_CONSTANTS.coldTempThreshold then
        carb.fuelEnrichment = CARBURETOR_CONSTANTS.warmUpEnrichment
        carb.idleMultiplier = CARBURETOR_CONSTANTS.coldIdleMultiplier
        carb.idleOffset = 150  -- RPM offset for cold
    elseif temp < CARBURETOR_CONSTANTS.warmTempThreshold then
        carb.fuelEnrichment = 1.0
        carb.idleMultiplier = CARBURETOR_CONSTANTS.warmIdleMultiplier
        carb.idleOffset = 100  -- RPM offset for warm
    else
        carb.fuelEnrichment = 1.0
        carb.idleMultiplier = CARBURETOR_CONSTANTS.hotIdleMultiplier
        carb.idleOffset = 50   -- RPM offset for hot
    end
    
    -- Initialize vaporization level if not set
    carb.vaporizationLevel = carb.vaporizationLevel or 0
    
    -- Handle fuel vaporization with safe defaults
    local vaporizationTempThreshold = CARBURETOR_CONSTANTS.vaporizationTempThreshold or 122  -- Default 50°C = 122°F
    local vaporizationRate = CARBURETOR_CONSTANTS.vaporizationRate or 0.1  -- Default rate
    
    -- Only proceed if we have a valid temperature
    if temp and type(temp) == 'number' then
        if temp > vaporizationTempThreshold then
            carb.vaporizationLevel = min(1.0, (carb.vaporizationLevel or 0) + vaporizationRate * dt)
        else
            carb.vaporizationLevel = max(0, (carb.vaporizationLevel or 0) - vaporizationRate * dt)
        end
    end
    
    -- Handle fuel starvation with safe defaults
    local starvationRecoveryTime = CARBURETOR_CONSTANTS.starvationRecoveryTime or 2.0  -- Default 2 seconds recovery time
    local starvationThreshold = CARBURETOR_CONSTANTS.starvationThreshold or 0.2  -- Default 20% float level threshold
    
    if carb.floatLevel < starvationThreshold then
        carb.starvationLevel = min(1.0, carb.starvationLevel + dt / starvationRecoveryTime)
        carb.isStarving = true
    else
        carb.starvationLevel = max(0, carb.starvationLevel - dt / starvationRecoveryTime)
        carb.isStarving = false
    end
    
    -- Handle engine surging
    local throttleChange = abs(throttle - (carb.lastThrottle or 0))
    if throttleChange > CARBURETOR_CONSTANTS.surgeThreshold then
        carb.isSurging = true
        carb.surgeTimer = CARBURETOR_CONSTANTS.surgeDuration
        
        -- Clear any existing surge sound
        if carb.surgeSound then
            if type(carb.surgeSound) == 'number' and obj.cutSFX then
                pcall(obj.cutSFX, obj, carb.surgeSound)
            end
            carb.surgeSound = nil
        end
        
        -- Only attempt to create sound if we have a valid engine node and sound functions exist
        if self.device and self.device.engineNodeID and obj.createSFXSource and obj.setSFXVolume and obj.playSFX then
            -- Use pcall to safely attempt sound creation
            local success, soundId = pcall(function()
                return obj:createSFXSource("event:>Engine>Power>Surge", "AudioDefaultLoop3D", "", self.device.engineNodeID, 0)
            end)
            
            -- If sound creation was successful, try to play it
            if success and soundId and soundId ~= 0 then
                -- Store the sound ID for later cleanup
                carb.surgeSound = soundId
                
                -- Safely set volume and play
                pcall(function()
                    obj:setSFXVolume(soundId, CARBURETOR_CONSTANTS.surgeSoundVolume or 1.0)
                    obj:playSFX(soundId)
                end)
            else
                -- If we got here, sound creation failed
                carb.surgeSound = nil
            end
        end
    end
    
    -- Update surge timer
    if carb.isSurging then
        carb.surgeTimer = carb.surgeTimer - dt
        if carb.surgeTimer <= 0 then
            carb.isSurging = false
            -- Safely clean up sound if it exists
            if carb.surgeSound then
                if type(carb.surgeSound) == 'number' and obj.cutSFX then
                    pcall(obj.cutSFX, obj, carb.surgeSound)
                end
                carb.surgeSound = nil
            end
        end
    end

    -- Apply idle settings
    if carb.device.throttle == 0 then  -- Only apply idle settings when throttle is at 0
        -- Get base idle speed with fallback to device's idleRPM if not set
        local baseIdle = carb.device.baseIdleSpeed or (carb.device.idleRPM or 800)  -- Default to 800 RPM if neither is set
        
        -- Convert RPM to AV (angular velocity)
        local targetAV = (baseIdle * (1 + (carb.idleMultiplier or 0)) + (carb.idleOffset or 0)) * rpmToAV
        
        -- Apply the idle speed override (do not zero idleAVStartOffset or idleAVReadError
        -- as those are managed by the engine's idle roughness simulation)
        carb.device.idleAVOverwrite = targetAV
    end
    
    -- Update fuel usage with surge multiplier
    local surgeMultiplier = carb.isSurging and CARBURETOR_CONSTANTS.surgeFuelMultiplier or 1.0
    
    -- Derive base fuel usage from engine state (RPM and throttle)
    -- device.spentEnergy is accumulated by combustionEngine each tick
    local rpm = abs(engineAV or 0) * avToRPM
    local baseFuelRate = (rpm / 1000) * (currentThrottle + 0.1) * 0.001
    self.device.fuelUsage = baseFuelRate * carb.fuelEnrichment * (1 + (carb.vaporizationLevel or 0) * 0.2) * surgeMultiplier
    
    -- Store current throttle for next frame
    carb.lastThrottle = throttle
end

-- Get choke state for engine
function carburetor:getChokeState(engineTempC, isCranking, isStarting)
    local isActive = self.chokeActive
    local effect = 0
    local throttleBoost = 0
    
    if isActive then
        if engineTempC < 20 then
            effect = CARBURETOR_CONSTANTS.chokeEffectCold
            throttleBoost = CARBURETOR_CONSTANTS.chokeThrottleBoostCold
        elseif engineTempC < 60 then
            local t = (engineTempC - 20) / 40
            effect = CARBURETOR_CONSTANTS.chokeEffectCold + t * (CARBURETOR_CONSTANTS.chokeEffectWarm - CARBURETOR_CONSTANTS.chokeEffectCold)
            throttleBoost = CARBURETOR_CONSTANTS.chokeThrottleBoostCold + t * (CARBURETOR_CONSTANTS.chokeThrottleBoostWarm - CARBURETOR_CONSTANTS.chokeThrottleBoostCold)
        end
    end
    
    -- Adjust effect based on pull-off
    if self.chokePullOffActive then
        effect = effect * 0.5
        throttleBoost = throttleBoost * 0.5
    end
    
    return {
        isActive = isActive,
        effect = effect,
        throttleBoost = throttleBoost
    }
end

-- Adjust final torque from carburetor
function carburetor:adjustFinalTorque(torque, params)
    local engineAV = params.engineAV
    local throttle = params.throttle
    
    -- Apply starvation effect
    if self.isStarving then
        torque = torque * (1 - self.starvationLevel)
    end
    
    -- Apply surge torque boost
    if self.isSurging then
        torque = torque * CARBURETOR_CONSTANTS.surgeTorqueMultiplier
    end
    
    -- Apply flooding effect (torque reduction)
    local floodLevel = self:getFloodLevel()
    if floodLevel > 0.1 then
        torque = torque * (1 - floodLevel * 0.8)
    end
    
    return torque
end

-- Update electrics with carburetor state
function carburetor:updateElectrics(electrics, params)
    electrics.values.engineFlooded = self:getFloodLevel() > 0.1
    electrics.values.chokePosition = self.chokeActive and 1 or 0
    electrics.values.fuelStarvation = self.isStarving
    electrics.values.floatLevel = self.floatLevel
end

-- Set choke state
function carburetor:setChoke(active)
    self.chokeActive = active
    self.chokeTimer = 0
end

-- Get fuel-related values
function carburetor:getFuelValues(engineTempC, isCranking)
    local values = {
        baseFuelAmount = CARBURETOR_CONSTANTS.baseFuelAmount,
        maxFuelPerCylinder = CARBURETOR_CONSTANTS.maxFuelPerCylinder,
        minFuelForInjection = CARBURETOR_CONSTANTS.minFuelForInjection,
        minFuelForCombustion = CARBURETOR_CONSTANTS.minFuelForCombustion,
        minAirForCombustion = CARBURETOR_CONSTANTS.minAirForCombustion,
        crankingFuelMultiplier = CARBURETOR_CONSTANTS.crankingFuelMultiplier
    }
    
    -- Apply temperature adjustments
    local tempFactor = math.max(0, math.min(1, (engineTempC - CARBURETOR_CONSTANTS.minTempForCombustion) / 
        CARBURETOR_CONSTANTS.tempAdjustmentRange))
    
    -- Adjust minimum fuel for combustion based on temperature
    values.minFuelForCombustion = values.minFuelForCombustion * (1.5 - tempFactor * 0.5)
    
    -- Adjust minimum air for combustion based on temperature
    values.minAirForCombustion = values.minAirForCombustion * (1.5 - tempFactor * 0.5)
    
    -- If cranking, adjust values
    if isCranking then
        values.minFuelForCombustion = values.minFuelForCombustion * 0.5
        values.minAirForCombustion = values.minAirForCombustion * 0.5
    end
    
    return values
end

-- Get fuel enrichment based on temperature and engine state
function carburetor:getFuelEnrichment(engineTempC, isCranking, throttle)
    local tempFactor = math.max(0, math.min(1, (engineTempC - CARBURETOR_CONSTANTS.minTempForCombustion) / 
        CARBURETOR_CONSTANTS.tempAdjustmentRange))
    
    -- Base enrichment based on temperature
    local enrichment = 1.0
    if engineTempC < 20 then
        -- Cold: use maximum enrichment
        enrichment = CARBURETOR_CONSTANTS.coldEnrichmentMax * (1 - tempFactor) + 1.0 * tempFactor
    elseif engineTempC < 80 then
        -- Warming up: reduce enrichment as engine warms
        local warmFactor = (engineTempC - 20) / 60
        enrichment = CARBURETOR_CONSTANTS.warmEnrichment * (1 - warmFactor) + 1.0 * warmFactor
    else
        -- Hot: use hot enrichment (may be <1 for leaner mixture)
        enrichment = CARBURETOR_CONSTANTS.hotEnrichment
    end
    
    -- Additional enrichment during cranking
    if isCranking then
        enrichment = enrichment * CARBURETOR_CONSTANTS.crankingFuelMultiplier
    end
    
    -- Slight enrichment at high throttle
    if throttle > 0.8 then
        enrichment = enrichment * (1 + (throttle - 0.8) * 0.2)
    end
    
    return math.max(0.5, math.min(4.0, enrichment))
end

-- Get the current flood level (0-1)
function carburetor:getFloodLevel()
    -- Calculate flood level based on float level and other factors
    local floodLevel = 0
    
    -- If float level is too high, increase flood level
    if self.floatLevel then
        local maxLevel = self.maxFloatLevel or CARBURETOR_CONSTANTS.maxFloatLevel
        local minLevel = self.minFloatLevel or CARBURETOR_CONSTANTS.minFloatLevel
        local normalRange = maxLevel - minLevel
        
        if normalRange > 0 and self.floatLevel > maxLevel then
            -- Flood level increases as float level rises above max
            floodLevel = math.min(1.0, (self.floatLevel - maxLevel) / (maxLevel * 0.5))
        end
    end
    
    -- If we're in a flooded state from the flooding system, use that value
    if self.floodingLevel and self.floodingLevel > floodLevel then
        floodLevel = self.floodingLevel
    end
    
    -- Apply vapor lock flood factor only at extreme temperatures (genuine vapor lock)
    if (self.vaporizationLevel or 0) > 0.5 then
        floodLevel = math.min(1.0, floodLevel + (self.vaporizationLevel - 0.5) * 0.2)
    end
    
    return floodLevel
end

-- Get the per-tick flood contribution for the per-cylinder system in combustionEngine.
-- Returns a small delta (not an absolute level) that gets distributed across cylinders.
function carburetor:getFloodContribution(dt)
    local contribution = 0
    dt = dt or 0.0005

    -- Float bowl overflow contributes to flooding
    if self.floatLevel then
        local maxLevel = self.maxFloatLevel or CARBURETOR_CONSTANTS.maxFloatLevel
        if self.floatLevel > maxLevel then
            contribution = contribution + math.min(0.02, (self.floatLevel - maxLevel) * 0.1) * dt
        end
    end

    -- Cranking-based flooding from the carburetor's own flooding tracking
    if self.floodingLevel and self.floodingLevel > 0.1 then
        contribution = contribution + self.floodingLevel * 0.005 * dt
    end

    -- Vapor lock contributes to flooding at extreme temperatures
    if (self.vaporizationLevel or 0) > 0.5 then
        contribution = contribution + (self.vaporizationLevel - 0.5) * 0.01 * dt
    end

    return contribution
end

-- Get idle multiplier based on carburetor state
function carburetor:getIdleMultiplier()
    local multiplier = 1.0
    
    -- Get engine temperature
    local tempC = self.device and self.device.thermals and self.device.thermals.engineBlockTemperature or 0
    local temp = (tempC * 9/5) + 32
    
    -- Apply temperature-based idle multiplier
    multiplier = multiplier * self:getTemperatureMultiplier(temp)
    
    -- Apply choke effect with gradual pull-off reduction
    -- Full choke boost at timer=0, reducing to 1.0 (no effect) at timer=chokePullOffTime
    if self.chokeActive then
        local params = self.params or CARBURETOR_CONSTANTS
        local chokeMultiplier = params.chokeIdleMultiplier or CARBURETOR_CONSTANTS.chokeIdleMultiplier
        local pullOffFraction = (self.chokePullOffTimer or 0) / CARBURETOR_CONSTANTS.chokePullOffTime
        multiplier = multiplier * (1 + (chokeMultiplier - 1) * (1 - pullOffFraction))
    end
    
    -- Apply starvation effect if active
    if self.isStarving then
        multiplier = multiplier * (1 - self.starvationLevel * 0.5)
    end
    
    return multiplier
end

return carburetor
