-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true} --set dynamically
M.deviceCategories = {engine = true}

local carburetor = require('powertrain/carburetor')
local thermals = require('powertrain/combustionEngineThermals')

local delayLine = rerequire("delayLine")

local max = math.max
local min = math.min
local abs = math.abs
local floor = math.floor
local random = math.random
local smoothmin = smoothmin

local rpmToAV = 0.104719755
local avToRPM = 9.549296596425384
local torqueToPower = 0.0001404345295653085
local psToWatt = 735.499
local hydrolockThreshold = 1.9

-- Warm-up and flooding constants
local WARMUP_PHASES = {
    cold = {temp = -20, idle = 250},
    warm = {temp = 0, idle = 200},
    normal = {temp = 20, idle = 150},
    hot = {temp = 60, idle = 100}
}

-- Constants for flooding behavior
local engineFloodValues = {
    threshold = 2.0,  -- seconds before engine is considered flooded
    clearTime = 10.0,  -- seconds for flood to clear completely
    idleMultiplier = 0.5,  -- Base multiplier for idle speed
    rpmMultiplier = 0.3,  -- Base multiplier for RPM fluctuation
    torqueMultiplier = 1.8,  -- Base multiplier for torque reduction
    vaporLockMultiplier = 0.5,  -- Additional multiplier for vapor lock
    maxStarterTorque = 500,  -- Maximum starter torque during flooding
    maxStarterRPM = 200,    -- Maximum starter RPM during flooding
    roughRunningIntensity = 0.5,  -- Base rough running intensity
    starterTorqueReduction = 0.65,  -- Torque reduction during flooding
    starterRPMLimit = 0.85,   -- RPM limit during flooding
    idlePenalty = 0.2,      -- Idle speed penalty during flooding
    vaporLockPenalty = 0.3,  -- Additional penalty for vapor lock
    minTorqueThreshold = 0.3, -- Minimum torque threshold (30% of normal)
    torqueTransitionTime = 2.0 -- Time to transition torque changes
}

-- Function to update engine-specific flooding values
local function updateEngineFloodValues(engine)
    if not engine then return end
    
    -- Update flooding values from engine configuration
    engineFloodValues.threshold = engine.floodThreshold or engineFloodValues.threshold
    engineFloodValues.clearTime = engine.floodClearTime or engineFloodValues.clearTime
    engineFloodValues.idleMultiplier = engine.floodIdleMultiplier or engineFloodValues.idleMultiplier
    engineFloodValues.rpmMultiplier = engine.floodRPMMultiplier or engineFloodValues.rpmMultiplier
    engineFloodValues.torqueMultiplier = engine.floodTorqueMultiplier or engineFloodValues.torqueMultiplier
    engineFloodValues.vaporLockMultiplier = engine.floodVaporLockMultiplier or engineFloodValues.vaporLockMultiplier
    engineFloodValues.maxStarterTorque = engine.floodMaxStarterTorque or engineFloodValues.maxStarterTorque
    engineFloodValues.maxStarterRPM = engine.floodMaxStarterRPM or engineFloodValues.maxStarterRPM
    engineFloodValues.roughRunningIntensity = engine.floodRoughRunningIntensity or engineFloodValues.roughRunningIntensity
    engineFloodValues.starterTorqueReduction = engine.floodStarterTorqueReduction or engineFloodValues.starterTorqueReduction
    engineFloodValues.starterRPMLimit = engine.floodStarterRPMLimit or engineFloodValues.starterRPMLimit
    engineFloodValues.idlePenalty = engine.floodIdlePenalty or engineFloodValues.idlePenalty
    engineFloodValues.vaporLockPenalty = engine.floodVaporLockPenalty or engineFloodValues.vaporLockPenalty
    engineFloodValues.minTorqueThreshold = engine.floodMinTorqueThreshold or engineFloodValues.minTorqueThreshold
    engineFloodValues.torqueTransitionTime = engine.floodTorqueTransitionTime or engineFloodValues.torqueTransitionTime
end

-- Vapor lock constants
--[[ local VAPOR_LOCK_THRESHOLD = 100  -- temperature in °C
local VAPOR_LOCK_CLEAR_TIME = 20.0  -- seconds to clear vapor lock
local VAPOR_LOCK_CHANCE = 0.3  -- chance of vapor lock per second above threshold
]]
local function getWarmUpIdle(device)
    local temp = device.thermals.engineBlockTemperature
    
    -- Determine current warm-up phase
    if temp < WARMUP_PHASES.cold.temp then
        return WARMUP_PHASES.cold.idle
    elseif temp < WARMUP_PHASES.warm.temp then
        local ratio = (temp - WARMUP_PHASES.cold.temp) / (WARMUP_PHASES.warm.temp - WARMUP_PHASES.cold.temp)
        return WARMUP_PHASES.cold.idle + ratio * (WARMUP_PHASES.warm.idle - WARMUP_PHASES.cold.idle)
    elseif temp < WARMUP_PHASES.normal.temp then
        local ratio = (temp - WARMUP_PHASES.warm.temp) / (WARMUP_PHASES.normal.temp - WARMUP_PHASES.warm.temp)
        return WARMUP_PHASES.warm.idle + ratio * (WARMUP_PHASES.normal.idle - WARMUP_PHASES.warm.idle)
    elseif temp < WARMUP_PHASES.hot.temp then
        local ratio = (temp - WARMUP_PHASES.normal.temp) / (WARMUP_PHASES.hot.temp - WARMUP_PHASES.normal.temp)
        return WARMUP_PHASES.normal.idle + ratio * (WARMUP_PHASES.hot.idle - WARMUP_PHASES.normal.idle)
    else
        return WARMUP_PHASES.hot.idle
    end
end

local function updateVaporLockState(device, dt)
    -- Check for vapor lock conditions
    if device.thermals.engineBlockTemperature > 100 then
        if not device.vaporLockTimer then
            device.vaporLockTimer = 0
            device.vaporLockLevel = 0
            device.isVaporLocked = false
        end
        
        device.vaporLockTimer = device.vaporLockTimer + dt
        
        -- Increase vapor lock level based on temperature
        local tempAboveThreshold = device.thermals.engineBlockTemperature - 100
        local chance = 0.3 * (1 + tempAboveThreshold / 100)
        
        if random() < chance * dt then
            device.isVaporLocked = true
            device.vaporLockLevel = 1.0
        end
    else
        if device.isVaporLocked then
            device.vaporLockTimer = 0
            device.vaporLockLevel = max(0, device.vaporLockLevel - dt / 10)
            if device.vaporLockLevel <= 0 then
                device.isVaporLocked = false
            end
        else
            device.vaporLockTimer = 0
        end
    end
end

local function updateFloodingState(device, dt)
  -- Initialize all flood-related variables if they don't exist
  device.floodTimer = device.floodTimer or 0
  device.floodLevel = device.floodLevel or 0
  device.isFlooded = device.isFlooded or false
  device.roughRunning = device.roughRunning or false
  device.roughRunningIntensity = device.roughRunningIntensity or 0
  device.lastFloodUpdate = device.lastFloodUpdate or 0
  device.floodRate = device.floodRate or 0  -- Rate of flooding accumulation
  device.clearRate = device.clearRate or 0  -- Rate of flooding clearance
  device.floodThreshold = device.floodThreshold or 0.5  -- Threshold for flooded state
  device.starterStress = device.starterStress or 0
  device.sparkFouling = device.sparkFouling or 0
  device.primingLevel = device.primingLevel or 0
  device.primingTimer = device.primingTimer or 0
  device.primingThreshold = device.primingThreshold or 0.3
  device.primingMax = device.primingMax or 0.3
  
  -- Throttle pump detection
  device.lastThrottle = device.lastThrottle or 0
  device.throttleDelta = device.throttleDelta or 0
  device.throttlePumpTimer = device.throttlePumpTimer or 0
  device.throttlePumpFuel = device.throttlePumpFuel or 0
  device.idleFloodTimer = device.idleFloodTimer or 0
  device.lastRPM = device.lastRPM or 0
  device.rpmDropTimer = device.rpmDropTimer or 0

  -- Update throttle delta for pump detection
  device.throttleDelta = math.abs(device.throttle - (device.lastThrottle or 0))
  device.lastThrottle = device.throttle
  
  -- Only update flooding state periodically for performance
  device.lastFloodUpdate = device.lastFloodUpdate + dt
  if device.lastFloodUpdate < 0.1 then  -- Update 10 times per second
    return
  end
  local dtScaled = device.lastFloodUpdate
  device.lastFloodUpdate = 0
  
  -- Get current RPM and temperature
  local currentRPM = device.outputAV1 * 30 / math.pi
  local tempC = device.thermals and (device.thermals.engineBlockTemperature - 273.15) or 20
  local isCold = tempC < 20
  
  -- Track RPM drops
  if currentRPM < device.lastRPM * 0.8 and currentRPM < device.idleRPM * 1.2 then
    device.rpmDropTimer = device.rpmDropTimer + dtScaled
  else
    device.rpmDropTimer = math.max(0, device.rpmDropTimer - dtScaled * 0.5)
  end
  device.lastRPM = currentRPM
  
  -- Handle throttle pumping effects
  if device.throttleDelta > 0.3 and device.throttle > 0.7 then
    device.throttlePumpTimer = 0.5  -- 500ms window for throttle pump effect
    -- Add extra fuel for throttle pump
    device.throttlePumpFuel = math.min(1.0, device.throttlePumpFuel + device.throttleDelta * 0.3)
  elseif device.throttlePumpTimer > 0 then
    device.throttlePumpTimer = math.max(0, device.throttlePumpTimer - dtScaled)
  else
    device.throttlePumpFuel = math.max(0, device.throttlePumpFuel - dtScaled * 0.2)
  end
  
  -- Apply throttle pump fuel enrichment
  if device.throttlePumpFuel > 0 then
    device.throttle = math.min(1.0, device.throttle + device.throttlePumpFuel * 0.5)
  end

  -- Handle idle flooding when engine is running
  if device.engineState and currentRPM < device.idleRPM * 1.2 and device.throttle < 0.2 then
    -- Accumulate idle flood timer when RPM is low and throttle is closed
    device.idleFloodTimer = device.idleFloodTimer + dtScaled
    
    -- Increase flood level if idling poorly for too long
    if device.idleFloodTimer > 5.0 and isCold then
      local floodAmount = (1 - (currentRPM / (device.idleRPM * 1.2))) * 0.01
      device.floodLevel = math.min(1.0, device.floodLevel + floodAmount)
      
      if device.floodLevel > 0.3 and device.floodLevel - floodAmount <= 0.3 then
        guihooks.message("Engine running rich at idle", 2.0, "vehicle.damage")
      end
    end
  else
    device.idleFloodTimer = math.max(0, device.idleFloodTimer - dtScaled * 0.5)
  end
  
  -- Handle RPM drops that could indicate flooding
  if device.rpmDropTimer > 1.0 and device.throttle < 0.3 then
    local floodAmount = (1 - (currentRPM / (device.lastRPM + 1))) * 0.1
    device.floodLevel = math.min(1.0, device.floodLevel + floodAmount)
    device.rpmDropTimer = 0
  end

  -- Handle throttle priming when engine is off
  if device.engineState == 0 and device.throttle > 0.1 then
    -- Increase priming level when throttle is pressed
    if device.primingTimer <= 0 then
      device.primingTimer = 0.2  -- 200ms between priming
      device.primingLevel = min(device.primingLevel + 0.1, device.primingMax)
      
      -- If priming level exceeds threshold, start flooding
      if device.primingLevel >= device.primingThreshold then
        device.floodLevel = min(1.0, device.floodLevel + 0.1)
        device.primingLevel = 0  -- Reset priming
        device.primingTimer = 0
        
        -- Show warning if flood level is getting high
        if device.floodLevel >= 0.7 then
          guihooks.message("Engine is flooding!", 2.0, "vehicle.damage")
        end
      end
    else
      device.primingTimer = device.primingTimer - dt
    end
  else
    -- Reduce priming level when throttle is released
    device.primingLevel = max(0, device.primingLevel - (dtScaled * 0.5))
    device.primingTimer = 0
  end

  -- Calculate temperature effects (colder = easier to flood)
  local tempFactor = 1.0
  if device.thermals and device.thermals.engineBlockTemperature then
    -- Enhanced temperature effect with steeper curve around freezing
    local tempC = device.thermals.engineBlockTemperature - 273.15  -- Convert K to °C
    if tempC < 0 then
      tempFactor = 0.3 + (tempC + 20) * 0.01  -- Very sensitive below freezing
    else
      tempFactor = 0.5 + (tempC / 25)  -- Gradual improvement above freezing
    end
    tempFactor = clamp(tempFactor, 0.2, 1.5)
    
    -- Additional flood risk when very cold and idling
    if tempC < 5 and device.engineState and device.outputAV1 < device.idleAV * 1.2 then
      device.floodLevel = math.min(1.0, device.floodLevel + (0.01 * (5 - tempC) * dtScaled * 0.1))
    end
  end

  -- Adjust flood thresholds based on temperature
  local floodThreshold = 0.25 * tempFactor  -- Lower threshold when cold
  local clearThreshold = 0.15 * tempFactor  -- Lower threshold when cold

  -- Handle flood clearing when engine is running
  if device.engineState then
    -- More effective drying when engine is running
    if device.floodLevel > 0 then
      -- Calculate clear rate based on engine state
      local baseClearRate = 0.05  -- Reduced base clear rate per second
      
      -- Get current RPM and temperature
      local rpm = device.outputAV1 * 30 / math.pi
      local tempC = device.thermals and (device.thermals.engineBlockTemperature - 273.15) or 20
      
      -- Slower clearing when cold
      local tempFactor = clamp((tempC + 10) / 30, 0.2, 1.5)  -- 0.2 at -10°C, 1.0 at 20°C, 1.5 at 35°C
      
      -- Adjust clear rate based on RPM and temperature
      local rpmMultiplier = 0.5  -- Base multiplier when cranking
      if rpm > device.idleRPM * 1.5 then
        -- Higher RPMs clear faster, but not as aggressively as before
        rpmMultiplier = clamp(0.5 + (rpm / 3000), 0.5, 2.5)
      end
      
      -- Apply all factors to clear rate
      device.clearRate = baseClearRate * rpmMultiplier * tempFactor
      
      -- Additional clear rate from spark fouling
      if device.slowIgnitionErrorCoef < 1.0 or device.fastIgnitionErrorCoef < 1.0 then
        -- Calculate spark fouling based on ignition error coefficients
        local foulingLevel = 1 - min(device.slowIgnitionErrorCoef, device.fastIgnitionErrorCoef)
        local foulingBonus = foulingLevel * 0.5
        device.clearRate = device.clearRate + foulingBonus
        
        -- Reduce ignition error timers based on fouling level
        if device.slowIgnitionErrorTimer > 0 then
          device.slowIgnitionErrorTimer = max(0, device.slowIgnitionErrorTimer - (foulingBonus * dt))
        end
        if device.fastIgnitionErrorTimer > 0 then
          device.fastIgnitionErrorTimer = max(0, device.fastIgnitionErrorTimer - (foulingBonus * dt))
        end
      end
      
      -- Apply clear rate based on calculated rate
      device.floodLevel = max(0, device.floodLevel - (device.clearRate * dt))
      if device.floodLevel == 0 then
        device.isFlooded = false
        device.roughRunning = false
        device.roughRunningIntensity = 0
      end
    end
    
    -- Don't return here anymore - we want to handle both running and non-running cases
  end
  
  -- Handle flooding when engine is off
  if device.engineState == 0 then
    -- Accumulate flood timer when starter is engaged
    if device.starterEngagedCoef == 1 then
      -- Calculate flood rate based on engine state
      local baseFloodRate = 0.03 * tempFactor  -- Base rate per second
      local stressMultiplier = 1.0
      if device.starterStress then
        stressMultiplier = 1 + device.starterStress * 0.7  -- Stress impact multiplier
      end
      
      -- Calculate flood rate based on engine RPM
      local rpmMultiplier = 1.0
      if device.outputAV1 then
        local rpm = device.outputAV1 * avToRPM
        rpmMultiplier = clamp(1.0 + (rpm / 1000), 1.0, 1.5)  -- Increase rate with RPM up to 50% at 1000 RPM
      end
      
      -- Calculate final flood rate
      local floodRate = baseFloodRate * stressMultiplier * rpmMultiplier
      
      -- Update flood level and timer
      device.floodLevel = min(1.0, device.floodLevel + (floodRate * dt))
      device.floodTimer = device.floodTimer + dt
      
      -- Increase starter stress during cranking
      device.starterStress = min(1.0, (device.starterStress or 0) + (dt / 10))
    else
      -- Cool down starter when not in use
      device.starterStress = max(0, (device.starterStress or 0) - (dt / 30))
    end
    
    -- Check for vapor lock affecting flooding
    if device.isVaporLocked then
      device.floodLevel = min(1.0, device.floodLevel + (0.01 * dt * (1 + device.vaporLockLevel)))
    end

    -- Update flood state with balanced threshold
    local wasFlooded = device.isFlooded
    device.isFlooded = device.floodLevel >= floodThreshold
    
    -- Add effects when flooded
    if device.isFlooded then
      -- Gradual increase in flood level
      device.floodLevel = min(1.0, device.floodLevel + (0.015 * dt))
      -- Balanced rough running intensity
      device.roughRunningIntensity = min(1.0, device.floodLevel * 1.3)
      
      -- Show warning if flood level is critical
      if device.floodLevel >= 0.8 then
        guihooks.message("Engine is severely flooded! Use full throttle to clear.", 2.0, "vehicle.damage")
      end
    else
      -- Gradual decrease in flood level when not flooded
      device.floodLevel = max(0, device.floodLevel - (0.01 * dt))
    end
  
    -- Only update if state changed
    if device.isFlooded ~= wasFlooded then
      if device.isFlooded then
        -- Just crossed into flooded state
        device.roughRunning = true
        device.roughRunningIntensity = 0.5
        device.sparkFouling = min(1.0, (device.sparkFouling or 0) + 0.3)
        
        -- Show warning
        guihooks.message("Engine is flooded!", 2.0, "vehicle.damage")
      else
        -- Just recovered from flooded state
        device.roughRunning = false
        device.roughRunningIntensity = 0
      end
    end
  end

  -- Handle flooding state
  if device.isFlooded then
    -- Increase rough running intensity based on flood level
    device.roughRunningIntensity = min(1.0, device.floodLevel * 1.2)
    
    -- Apply spark fouling effects
    if device.sparkFouling > 0.5 and device.engineState then
      -- Random misfires when spark plugs are fouled
      if math.random() < (device.sparkFouling - 0.3) * 0.1 then
        local misfireSeverity = 0.85 + math.random() * 0.15  -- 15-30% RPM drop
        device.outputAV1 = device.outputAV1 * misfireSeverity
        
        -- Add visual effect for severe misfires
        if misfireSeverity < 0.9 and device.triggerMisfireEffect then
          device.triggerMisfireEffect(device, 1.0 - misfireSeverity)
        end
      end
    end
    
    -- Update UI with flood status
    if device.updateFloodStatus then
      local status = {
        isFlooded = true,
        floodLevel = device.floodLevel,
        starterStress = device.starterStress or 0,
        sparkFouling = device.sparkFouling or 0
      }
      device.updateFloodStatus(status)
    end
    
    -- Play flooding sound effects
    if device.playFloodingSound and device.lastFloodSoundTime and (device.lastFloodSoundTime + 2.0) < device.floodTimer then
      if math.random() < 0.3 then  -- 30% chance to play a flooding sound every 2 seconds
        local soundType = "misfire"
        if device.floodLevel > 0.7 then
          soundType = "flooded"
        elseif device.floodLevel > 0.4 then
          soundType = "struggling"
        end
      end
    end
  end
end

local function getTorqueData(device)
    local curves = {}
    local curveCounter = 1
    local maxTorque = 0
    local maxTorqueRPM = 0
    local maxPower = 0
    local maxPowerRPM = 0
    local maxRPM = device.maxRPM

  local turboCoefs = nil
  local superchargerCoefs = nil
  local nitrousTorques = nil

  local torqueCurve = {}
  local powerCurve = {}

  for k, v in pairs(device.torqueCurve) do
    if type(k) == "number" and k < maxRPM then
      torqueCurve[k + 1] = v - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
      powerCurve[k + 1] = torqueCurve[k + 1] * k * torqueToPower
      if torqueCurve[k + 1] > maxTorque then
        maxTorque = torqueCurve[k + 1]
        maxTorqueRPM = k + 1
      end
      if powerCurve[k + 1] > maxPower then
        maxPower = powerCurve[k + 1]
        maxPowerRPM = k + 1
      end
    end
  end

  table.insert(curves, curveCounter, {torque = torqueCurve, power = powerCurve, name = "NA", priority = 10})

  if device.nitrousOxideInjection.isExisting then
    local torqueCurveNitrous = {}
    local powerCurveNitrous = {}
    nitrousTorques = device.nitrousOxideInjection.getAddedTorque()

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveNitrous[k + 1] = v + (nitrousTorques[k] or 0) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveNitrous[k + 1] = torqueCurveNitrous[k + 1] * k * torqueToPower
        if torqueCurveNitrous[k + 1] > maxTorque then
          maxTorque = torqueCurveNitrous[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveNitrous[k + 1] > maxPower then
          maxPower = powerCurveNitrous[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveNitrous, power = powerCurveNitrous, name = "N2O", priority = 20})
  end

  if device.turbocharger.isExisting then
    local torqueCurveTurbo = {}
    local powerCurveTurbo = {}
    turboCoefs = device.turbocharger.getTorqueCoefs()

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveTurbo[k + 1] = (v * (turboCoefs[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveTurbo[k + 1] = torqueCurveTurbo[k + 1] * k * torqueToPower
        if torqueCurveTurbo[k + 1] > maxTorque then
          maxTorque = torqueCurveTurbo[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveTurbo[k + 1] > maxPower then
          maxPower = powerCurveTurbo[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveTurbo, power = powerCurveTurbo, name = "Turbo", priority = 30})
  end

  if device.supercharger.isExisting then
    local torqueCurveSupercharger = {}
    local powerCurveSupercharger = {}
    superchargerCoefs = device.supercharger.getTorqueCoefs()

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveSupercharger[k + 1] = (v * (superchargerCoefs[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveSupercharger[k + 1] = torqueCurveSupercharger[k + 1] * k * torqueToPower
        if torqueCurveSupercharger[k + 1] > maxTorque then
          maxTorque = torqueCurveSupercharger[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveSupercharger[k + 1] > maxPower then
          maxPower = powerCurveSupercharger[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveSupercharger, power = powerCurveSupercharger, name = "SC", priority = 40})
  end

  if device.turbocharger.isExisting and device.supercharger.isExisting then
    local torqueCurveFinal = {}
    local powerCurveFinal = {}

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveFinal[k + 1] = (v * (turboCoefs[k] or 0) * (superchargerCoefs[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveFinal[k + 1] = torqueCurveFinal[k + 1] * k * torqueToPower
        if torqueCurveFinal[k + 1] > maxTorque then
          maxTorque = torqueCurveFinal[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveFinal[k + 1] > maxPower then
          maxPower = powerCurveFinal[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveFinal, power = powerCurveFinal, name = "Turbo + SC", priority = 50})
  end

  if device.turbocharger.isExisting and device.nitrousOxideInjection.isExisting then
    local torqueCurveFinal = {}
    local powerCurveFinal = {}

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveFinal[k + 1] = (v * (turboCoefs[k] or 0) + (nitrousTorques[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveFinal[k + 1] = torqueCurveFinal[k + 1] * k * torqueToPower
        if torqueCurveFinal[k + 1] > maxTorque then
          maxTorque = torqueCurveFinal[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveFinal[k + 1] > maxPower then
          maxPower = powerCurveFinal[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveFinal, power = powerCurveFinal, name = "Turbo + N2O", priority = 60})
  end

  if device.supercharger.isExisting and device.nitrousOxideInjection.isExisting then
    local torqueCurveFinal = {}
    local powerCurveFinal = {}

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveFinal[k + 1] = (v * (superchargerCoefs[k] or 0) + (nitrousTorques[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveFinal[k + 1] = torqueCurveFinal[k + 1] * k * torqueToPower
        if torqueCurveFinal[k + 1] > maxTorque then
          maxTorque = torqueCurveFinal[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveFinal[k + 1] > maxPower then
          maxPower = powerCurveFinal[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveFinal, power = powerCurveFinal, name = "SC + N2O", priority = 70})
  end

  if device.turbocharger.isExisting and device.supercharger.isExisting and device.nitrousOxideInjection.isExisting then
    local torqueCurveFinal = {}
    local powerCurveFinal = {}

    for k, v in pairs(device.torqueCurve) do
      if type(k) == "number" and k < maxRPM then
        torqueCurveFinal[k + 1] = (v * (turboCoefs[k] or 0) * (superchargerCoefs[k] or 0) + (nitrousTorques[k] or 0)) - device.friction * device.wearFrictionCoef * device.damageFrictionCoef - (device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef * k * rpmToAV)
        powerCurveFinal[k + 1] = torqueCurveFinal[k + 1] * k * torqueToPower
        if torqueCurveFinal[k + 1] > maxTorque then
          maxTorque = torqueCurveFinal[k + 1]
          maxTorqueRPM = k + 1
        end
        if powerCurveFinal[k + 1] > maxPower then
          maxPower = powerCurveFinal[k + 1]
          maxPowerRPM = k + 1
        end
      end
    end

    curveCounter = curveCounter + 1
    table.insert(curves, curveCounter, {torque = torqueCurveFinal, power = powerCurveFinal, name = "Turbo + SC + N2O", priority = 80})
  end

  table.sort(
    curves,
    function(a, b)
      local ra, rb = a.priority, b.priority
      if ra == rb then
        return a.name < b.name
      else
        return ra > rb
      end
    end
  )

  local dashes = {nil, {10, 4}, {8, 3, 4, 3}, {6, 3, 2, 3}, {5, 3}}
  for k, v in ipairs(curves) do
    v.dash = dashes[k]
    v.width = 2
  end

  return {maxRPM = maxRPM, curves = curves, maxTorque = maxTorque, maxPower = maxPower, maxTorqueRPM = maxTorqueRPM, maxPowerRPM = maxPowerRPM, finalCurveName = 1, deviceName = device.name, vehicleID = obj:getId()}
end

local function sendTorqueData(device, data)
  if not data then
    data = device:getTorqueData()
  end
  guihooks.trigger("TorqueCurveChanged", data)
end

local function scaleFrictionInitial(device, friction)
  device.friction = device.initialFriction * friction
end

local function scaleFriction(device, friction)
  device.friction = device.friction * friction
end

local function scaleOutputTorque(device, state, maxReduction)
  --scale torque ouput to some minimum, but do not let that minimum increase the actual scale (otherwise a min of 0.2 could "revive" and engine that sits at 0 scale already)
  device.outputTorqueState = max(device.outputTorqueState * state, min(maxReduction or 0, device.outputTorqueState))
  damageTracker.setDamage("engine", "engineReducedTorque", device.outputTorqueState < 1)
end

local function disable(device)
  device.outputTorqueState = 0
  device.isDisabled = false
  device.starterDisabled = false
  if device.starterEngagedCoef > 0 then
    device.starterEngagedCoef = 0
    obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
    end
  end

  damageTracker.setDamage("engine", "engineDisabled", true)
end

local function enable(device)
  device.outputTorqueState = 1
  device.isDisabled = false
  device.starterDisabled = false

  damageTracker.setDamage("engine", "engineDisabled", false)
end

local function lockUp(device)
  device.outputTorqueState = 1
  device.outputAVState = 1
  device.isDisabled = false
  device.isBroken = false
  device.starterDisabled = false
  if device.starterEngagedCoef > 0 then
    device.starterEngagedCoef = 0.3
    obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
    end
  end
  damageTracker.setDamage("powertrain", device.name, true)
  damageTracker.setDamage("engine", "engineLockedUp", true)
end

local function updateSounds(device, dt)
  local rpm = device.soundRPMSmoother:get(abs(device.outputAV1 * avToRPM), dt)
  local maxCurrentTorque = (device.torqueCurve[floor(rpm)] or 1) * device.intakeAirDensityCoef
  local engineLoad = device.soundLoadSmoother:get(device.instantEngineLoad, dt)
  local baseLoad = 0.3 * min(device.idleTorque / maxCurrentTorque, 1)
  engineLoad = max(engineLoad - baseLoad, 0) / (1 - baseLoad)
  local volumeCoef = rpm > 0.1 and device.engineVolumeCoef or 0

  if device.engineSoundID then
    local scaledEngineLoad = engineLoad * (device.soundMaxLoadMix - device.soundMinLoadMix) + device.soundMinLoadMix
    local fundamentalFreq = sounds.hzToFMODHz(rpm * device.soundConfiguration.engine.params.fundamentalFrequencyRPMCoef)
    obj:setEngineSound(device.engineSoundID, rpm, scaledEngineLoad, fundamentalFreq, volumeCoef)
  end

  if device.engineSoundIDExhaust then
    local minLoad = device.soundMinLoadMixExhaust or device.soundMinLoadMix
    local scaledEngineLoadExhaust = engineLoad * ((device.soundMaxLoadMixExhaust or device.soundMaxLoadMix) - minLoad) + minLoad
    local fundamentalFreqExhaust = sounds.hzToFMODHz(rpm * device.soundConfiguration.exhaust.params.fundamentalFrequencyRPMCoef)
    obj:setEngineSound(device.engineSoundIDExhaust, rpm, scaledEngineLoadExhaust, fundamentalFreqExhaust, volumeCoef)
  end

  device.turbocharger.updateSounds()
  device.supercharger.updateSounds()
end

local function checkHydroLocking(device, dt)
  if device.floodLevel > hydrolockThreshold then
    return
  end

  -- engine starts flooding if ALL of the waterDamage nodes are underwater
  local isFlooding = device.canFlood
  for _, n in ipairs(device.waterDamageNodes) do
    isFlooding = isFlooding and obj:inWater(n)
    if not isFlooding then
      break
    end
  end

  damageTracker.setDamage("engine", "engineIsHydrolocking", isFlooding)

  -- Initialize flood level if not set
  device.floodLevel = device.floodLevel or 0
  
  -- Base flood rates
  local floodRate = 0
  
  -- Check engine state
  local isEngineRunning = device.outputAV1 > device.idleAV * 0.8  -- Consider running if above 80% of idle RPM
  local isBelowIdle = device.outputAV1 > 0 and device.outputAV1 < device.idleAV * 1.2  -- Below 120% of idle RPM
  
  -- Increase flood level when:
  -- 1. Starter is engaged (cranking)
  -- 2. Engine is running but below idle (fouling plugs)
  if device.starterEngagedCoef > 0.5 then
    -- Cranking - faster flood rate
    floodRate = 0.15 * device.starterEngagedCoef
  elseif isEngineRunning and isBelowIdle then
    -- Running but below idle - slower flood rate (fouling plugs)
    local rpmRatio = 1 - (device.outputAV1 / device.idleAV)  -- 1 at 0 RPM, 0 at idle RPM
    floodRate = 0.03 * rpmRatio  -- Scale flood rate based on how far below idle we are
  end
  
  -- Decrease flood level when engine is running above idle
  if isEngineRunning and not isBelowIdle then
    -- Drying rate increases with RPM
    local rpmRatio = (device.outputAV1 - device.idleAV) / (device.idleAV * 2)  -- 0 at idle, 0.5 at 2x idle
    floodRate = -0.05 * (1 + rpmRatio * 2)  -- -0.05 at idle, up to -0.15 at high RPM
  end
  
  -- Update flood level with the calculated rate
  device.floodLevel = min(1, max(0, device.floodLevel + dt * floodRate))
  
  -- Misfire chance when engine is running and flooded
  if device.floodLevel > 0.1 and device.outputAV1 > device.idleAV * 0.5 then
    -- Higher flood level = higher chance of misfire
    local misfireChance = device.floodLevel * 0.05 * dt  -- 0-5% chance per second when fully flooded
    if math.random() < misfireChance then
      -- Create misfire effect using ignition error timers
      device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer + dt * 0.5  -- Increase slow error timer
      device.fastIgnitionErrorTimer = device.fastIgnitionErrorTimer + dt * 0.5  -- Increase fast error timer
      
      -- Reduce RPM slightly during misfire
      device.outputAV1 = device.outputAV1 * 0.8  -- Reduce RPM by 20%
      
      -- Add some flood level
      device.floodLevel = min(1, device.floodLevel + 0.05)
      
      -- Show misfire message occasionally
      if math.random(0, 1) < 0.1 then
        local msg = "Engine misfiring due to flooding!"
        if device.floodLevel > 0.7 then
          msg = "Engine sputtering badly!"
        end
        guihooks.message(msg, 2, "vehicle.damage")
      end
      
      -- If flood level is very high, eventually stall
      if device.floodLevel > 0.8 and math.random(0, 0.5) < 0.01 then
        device.outputAV1 = 0
        device.floodLevel = min(1, device.floodLevel + 0.1)
        guihooks.message("Engine stalled due to severe flooding!", 3, "vehicle.damage")
      end
    end
  end

  if device.floodLevel > hydrolockThreshold then
    damageTracker.setDamage("engine", "engineHydrolocked", true)
    device.outputAV1 = 0  -- Ensure engine stops
    guihooks.message("Engine hydrolocked! Turn off ignition and wait.", 4, "vehicle.damage.flood")
    return
  end

  -- Old flooding UI logic - replaced by new implementation in updateGFX
  --[[
  local currPercent = floor(0.01 + device.floodLevel * 100)
  
  -- Only update UI when percentage changes or when completely dry
  if currPercent ~= device.prevFloodPercent or (device.floodLevel <= 0 and device.prevFloodPercent > 0) then
    if currPercent > device.prevFloodPercent then
      -- Only show flooding messages when actually flooding
      if isFlooding then
        guihooks.message({txt = "vehicle.combustionEngine.engineFlooding", context = {percent = currPercent}}, 4, "vehicle.damage.flood")
      end
    else
      -- When drying, ensure we show the 0% message
      if device.floodLevel <= 1 then  -- Use a small threshold instead of exact 0
        device.floodLevel = 0  -- Ensure flood level is exactly 0 when dried
        guihooks.message("vehicle.combustionEngine.engineDried", 4, "vehicle.damage.flood")
      elseif currPercent < device.prevFloodPercent then
        guihooks.message({txt = "vehicle.combustionEngine.engineDrying", context = {percent = currPercent}}, 4, "vehicle.damage.flood")
      end
    end
  end
  device.prevFloodPercent = currPercent
  --]]
end

local function updateEnergyStorageRatios(device)
  for _, s in pairs(device.registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage and storage.energyType == device.requiredEnergyType then
      if storage.storedEnergy > 0 then
        device.energyStorageRatios[storage.name] = 1 / device.storageWithEnergyCounter
      else
        device.energyStorageRatios[storage.name] = 0
      end
    end
  end
end

local function updateFuelUsage(device)
  if not device.energyStorage then
    return
  end

  local hasFuel = false
  local previousTankCount = device.storageWithEnergyCounter
  local remainingFuelRatio = 0
  for _, s in pairs(device.registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage and storage.energyType == device.requiredEnergyType then
      local previous = device.previousEnergyLevels[storage.name]
      storage.storedEnergy = max(storage.storedEnergy - (device.spentEnergy * device.energyStorageRatios[storage.name]), 0)
      if previous > 0 and storage.storedEnergy <= 0 then
        device.storageWithEnergyCounter = device.storageWithEnergyCounter - 1
      elseif previous <= 0 and storage.storedEnergy > 0 then
        device.storageWithEnergyCounter = device.storageWithEnergyCounter + 1
      end
      device.previousEnergyLevels[storage.name] = storage.storedEnergy
      hasFuel = hasFuel or storage.storedEnergy > 0
      remainingFuelRatio = remainingFuelRatio + storage.remainingRatio
    end
  end

  if previousTankCount ~= device.storageWithEnergyCounter then
    device:updateEnergyStorageRatios()
  end

  if not hasFuel and device.hasFuel then
    device:disable()
  elseif hasFuel and not device.hasFuel then
    device:enable()
  end

  device.hasFuel = hasFuel
  device.remainingFuelRatio = remainingFuelRatio / device.storageWithEnergyCounter
end

local function updateGFX(device, dt)
  -- <<< Stall Buzzer Logic >>>
  if device.stallBuzzerSoundID then -- Check if the source was created successfully at init
    -- Condition: Ignition is ON, but engine RPM is below a threshold (e.g., 80% of idle)
    local shouldBuzzerBeActive = (device.ignitionCoef > 0) and (device.outputAV1 < device.idleAV * 0.8)

    -- Start/Stop the buzzer based on state change
    if shouldBuzzerBeActive and not device.stallBuzzerActive then
        obj:playSFX(device.stallBuzzerSoundID) -- Play the persistent source
        device.stallBuzzerActive = true
        -- log('D', 'StallBuzzer', 'Buzzer ON') -- Optional debug
    elseif not shouldBuzzerBeActive and device.stallBuzzerActive then
        obj:stopSFX(device.stallBuzzerSoundID) -- Stop the persistent source
        device.stallBuzzerActive = false
        -- log('D', 'StallBuzzer', 'Buzzer OFF') -- Optional debug
    end

    -- Adjust pitch if the buzzer is active
    if device.stallBuzzerActive then
        local targetPitch = 1.0
        if device.starterEngagedCoef > 0 then
            -- Lower pitch slightly when starter is cranking
            targetPitch = 1.0 - device.stallBuzzerCrankingPitch
        end
        obj:setPitch(device.stallBuzzerSoundID, targetPitch) -- Set pitch every frame while active
    end
end
    -- <<< End Stall Buzzer Logic >>>
    
    -- Update carburetor state
    if device.carburetor then
        device.carburetor:update(dt)
    end

    -- Add rough running effect when flooded
    if device.isFlooded and device.engineState then
        -- Make engine run rough with random RPM fluctuations
        local roughness = device.roughRunningIntensity
        local fluctuation = random(-1, 1) * roughness * device.idleAV * engineFloodValues.rpmFluctuation
        device.outputAV1 = device.outputAV1 + fluctuation
        
        -- Add rough running sound effect
        if device.engineMiscSounds.starterSoundEngine then
            local roughVolume = 0.8 + (random() * 0.2)  -- 80-100% volume
            obj:setVolume(device.engineMiscSounds.starterSoundEngine, roughVolume)
        end
    end
    
    -- Update vapor lock and flooding states
    updateVaporLockState(device, dt)
    updateFloodingState(device, dt)
    
    device:updateFuelUsage()

  device.outputRPM = device.outputAV1 * avToRPM

  device.starterThrottleKillTimer = max(device.starterThrottleKillTimer - dt, 0)
  device.lastStarterThrottleKillTimerEnd = max((device.lastStarterThrottleKillTimerEnd or 0) - dt*0.1, 0)

  if device.starterEngagedCoef > 0 then
    if device.starterBattery then
      local starterSpentEnergy = 1 / guardZero(abs(device.outputAV1)) * dt * device.starterTorque / 0.5 --0.5 efficiency
      device.starterBattery.storedEnergy = device.starterBattery.storedEnergy - starterSpentEnergy
      print(starterSpentEnergy)
      print(device.starterBattery.remainingRatio)
    end

  --  device.starterThrottleKillCoef = 1-device.starterThrottleKillTimer / device.starterThrottleKillTimerStart + math.max(linearScale(device.starterThrottleKillTimer, device.starterThrottleKillTimerStart, 0, 0, 3), 0.2)-0.2

    local killCoefFac = 1
    if device.starterThrottleKillTimer > 0 then
      killCoefFac = 1 - device.starterThrottleKillTimer/device.starterThrottleKillTimerStart
      -- Base starter ignition error chance increases as kill timer progresses (simulates flooding/heat soak)
      device.starterIgnitionErrorChance = killCoefFac * 6 * linearScale(device.thermals.engineBlockTemperature, -50, 0, 1, 0)
      killCoefFac = math.pow(killCoefFac, 8)*1.25
    else
        device.starterIgnitionErrorChance = 0 -- No error chance if kill timer is done
    end
    device.starterThrottleKillCoef = device.starterThrottleKillCoefSmoother:get(killCoefFac, dt)

    -- use lower starter max av multiplier in case the engine just doesnt start
    -- occasionally this would result in the engine starting and immediately shutting down, so its disabled
    local starterMaxAVMultiplier = 1.1 --math.min(1.1, device.outputAV1/device.starterMaxAV+(device.starterThrottleKillTimer == 0 and 0 or math.huge))

    local pitch = math.max(0.2,device.outputAV1/(device.starterMaxAV*starterMaxAVMultiplier))
    obj:setPitch(device.engineMiscSounds.starterSoundEngine, pitch)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:setPitch(device.engineMiscSounds.starterSoundExhaust, pitch)
    end

    if device.outputAV1 > device.starterMaxAV * starterMaxAVMultiplier then
      device.starterThrottleKillTimer = 0
      device.starterEngagedCoef = 0
      device.starterThrottleKillCoef = 0
      device.starterThrottleKillCoefSmoother:set(device.starterThrottleKillCoef)
      device.starterDisabled = false
      device.starterIgnitionErrorChance = 0
      obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
      if device.engineMiscSounds.starterSoundExhaust then
        obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
      end
    end
  end

  -- Get battery state from our simulation
  local batteryState = {}
  if BetterStarter and BetterStarter.getBatteryState then
    batteryState = BetterStarter.getBatteryState() or {}
  end
  
  -- Base battery effect (dimming as battery drains)
  local batteryEffect = 1
  if batteryState.stateOfCharge then
    -- Dim lights more aggressively below 50% charge
    batteryEffect = math.max(0.3, batteryState.stateOfCharge * 0.5 + 0.5)
    
    -- Additional dimming when voltage is low
    if batteryState.voltage then
      local voltageEffect = linearScale(batteryState.voltage, 10.5, 12.0, 0.5, 1)
      batteryEffect = math.min(batteryEffect, voltageEffect)
    end
  end

  -- Starter effect (dimming during cranking)
  local starterEffect = linearScale(device.starterEngagedCoef, 0, 1, 1, 0.3)
  
  -- RPM effect (brighter at higher RPMs)
  local rpmEffect = linearScale(device.outputRPM, 0, device.maxRPM, 0.8, 1)
  
  -- Pulsing effect (more noticeable flicker based on engine speed)
  -- Convert AV to RPM (AV * 9.549)
  local currentRPM = device.outputAV1 * avToRPM
  -- Increase base frequency for more noticeable pulsing
  local pulseFrequency = math.max(currentRPM / device.maxRPM, 0.05)
  -- Increase amplitude for more visible effect
  local pulseAmplitude = 0.05
  -- Adjust frequency multiplier and add some harmonic content for more interesting pattern
  local basePulse = math.sin(currentRPM * 0.2 * math.pi)
  local harmonic = math.sin(currentRPM * 0.4 * math.pi) * 0.7
  local pulseEffect = 1 + pulseAmplitude * (basePulse + harmonic)
  
  -- Combine all effects
  local baseLoadCoef = math.min(starterEffect, rpmEffect * pulseEffect)
  electrics.values.electricalLoadCoef = baseLoadCoef * batteryEffect
  
  -- Store battery voltage for other systems
  if batteryState.voltage then
    electrics.values.batteryVoltage = batteryState.voltage
  end


  device.starterIgnitionErrorTimer = device.starterIgnitionErrorTimer - dt
  if device.starterIgnitionErrorTimer <= 0 then
    device.starterIgnitionErrorTimer = math.random(device.starterIgnitionErrorInterval) * 0.1
    device.starterIgnitionErrorActive = math.random() < device.starterIgnitionErrorChance
  end

  device.starterIgnitionErrorCoef = 1
  if device.starterIgnitionErrorActive then
    device.starterIgnitionErrorCoef = device.starterIgnitionErrorSmoother:getUncapped(math.random(), dt)
  end

  device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer - dt
  if device.slowIgnitionErrorTimer <= 0 then
    device.slowIgnitionErrorTimer = math.random(device.slowIgnitionErrorInterval) * 0.1
    device.slowIgnitionErrorActive = math.random() < device.slowIgnitionErrorChance
  end

  device.slowIgnitionErrorCoef = 1
  if device.slowIgnitionErrorActive then
    device.slowIgnitionErrorCoef = device.slowIgnitionErrorSmoother:getUncapped(math.random(), dt)
  end

  local lowFuelIgnitionErrorChance = linearScale(device.remainingFuelRatio, 0.01, 0, 0, 0.4)
  local fastIgnitionErrorCoef = device.fastIgnitionErrorSmoother:getUncapped(math.random(), dt)
  device.fastIgnitionErrorCoef = fastIgnitionErrorCoef < (device.fastIgnitionErrorChance + lowFuelIgnitionErrorChance) and 0 or 1

  if device.shutOffSoundRequested and device.outputAV1 < device.idleAV * 0.95 and device.outputAV1 > device.idleAV * 0.5 then
    device.shutOffSoundRequested = false

    if device.engineMiscSounds.shutOffSoundEngine then
      obj:cutSFX(device.engineMiscSounds.shutOffSoundEngine)
      obj:playSFX(device.engineMiscSounds.shutOffSoundEngine)
    end

    if device.engineMiscSounds.shutOffSoundExhaust then
      obj:cutSFX(device.engineMiscSounds.shutOffSoundExhaust)
      obj:playSFX(device.engineMiscSounds.shutOffSoundExhaust)
    end
  end

  if device.outputAV1 < device.starterMaxAV * 0.8 and device.ignitionCoef > 0 then
    device.stallTimer = max(device.stallTimer - dt, 0)
    if device.stallTimer <= 0 and not device.isStalled then
      device.isStalled = true
    end
  else
    device.isStalled = false
    device.stallTimer = 1
  end

  device.revLimiterWasActiveTimer = min(device.revLimiterWasActiveTimer + dt, 1000)

  local rpmTooHigh = abs(device.outputAV1) > device.maxPhysicalAV
  damageTracker.setDamage("engine", "overRevDanger", rpmTooHigh)
  if rpmTooHigh then
    device.overRevDamage = min(max(device.overRevDamage + (abs(device.outputAV1) - device.maxPhysicalAV) * dt / device.maxOverRevDamage, 0), 1)
    local lockupChance = random(60, 100) * 0.01
    local valveHitChance = random(10, 60) * 0.01
    if lockupChance <= device.overRevDamage and not damageTracker.getDamage("engine", "catastrophicOverrevDamage") then
      device:lockUp()
      damageTracker.setDamage("engine", "catastrophicOverrevDamage", true)
      guihooks.message({txt = "vehicle.combustionEngine.engineCatastrophicOverrevDamage", context = {}}, 4, "vehicle.damage.catastrophicOverrev")

      if #device.engineBlockNodes >= 2 then
        sounds.playSoundOnceFollowNode("event:>Vehicle>Failures>engine_explode", device.engineBlockNodes[1], 1)

        for i = 1, 50 do
          local rnd = random()
          obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd, 43, 0, 1)
          obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd, 39, 0, 1)
          obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], -i * rnd, 43, 0, 1)
          obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], -i * rnd, 39, 0, 1)
        end
      end
    end
    if valveHitChance <= device.overRevDamage then
      device:scaleOutputTorque(0.98, 0.2)
      damageTracker.setDamage("engine", "mildOverrevDamage", true)
      guihooks.message({txt = "vehicle.combustionEngine.engineMildOverrevDamage", context = {}}, 4, "vehicle.damage.mildOverrev")
    end
  end

  if device.maxTorqueRating > 0 then
    damageTracker.setDamage("engine", "overTorqueDanger", device.combustionTorque > device.maxTorqueRating)
    if device.combustionTorque > device.maxTorqueRating then
      local torqueDifference = device.combustionTorque - device.maxTorqueRating
      device.overTorqueDamage = min(device.overTorqueDamage + torqueDifference * dt, device.maxOverTorqueDamage)
      if device.overTorqueDamage >= device.maxOverTorqueDamage and not damageTracker.getDamage("engine", "catastrophicOverTorqueDamage") then
        device:lockUp()
        damageTracker.setDamage("engine", "catastrophicOverTorqueDamage", true)
        guihooks.message({txt = "vehicle.combustionEngine.engineCatastrophicOverTorqueDamage", context = {}}, 4, "vehicle.damage.catastrophicOverTorque")

        if #device.engineBlockNodes >= 2 then
          sounds.playSoundOnceFollowNode("event:>Vehicle>Failures>engine_explode", device.engineBlockNodes[1], 1)

          for i = 1, 3 do
            local rnd = random()
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd * 3, 43, 0, 9)
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd * 3, 39, 0, 9)
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], -i * rnd * 3, 43, 0, 9)
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], -i * rnd * 3, 39, 0, 9)

            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd * 3, 56, 0, 1)
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd * 3, 57, 0, 1)
            obj:addParticleByNodesRelative(device.engineBlockNodes[2], device.engineBlockNodes[1], i * rnd * 3, 58, 0, 1)
          end
        end
      end
    end
  end

  --calculate the actual current idle torque to check for lockup conditions due to high friction
  local idleThrottle = device.maxIdleThrottle
  local idleTorque = (device.torqueCurve[floor(abs(device.idleAV) * avToRPM)] or 0) * device.intakeAirDensityCoef
  local idleThrottleMap = min(max(idleThrottle + idleThrottle * device.maxPowerThrottleMap / (idleTorque * device.forcedInductionCoef * abs(device.outputAV1) + 1e-30) * (1 - idleThrottle), 0), 1)
  idleTorque = ((idleTorque * device.forcedInductionCoef * idleThrottleMap) + device.nitrousOxideTorque)

  local finalFriction = device.friction * device.wearFrictionCoef * device.damageFrictionCoef
  local finalDynamicFriction = device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef
  local frictionTorque = finalFriction - (finalDynamicFriction * device.idleAV)

  if not device.isDisabled and (frictionTorque > device.maxTorque or (device.outputAV1 < device.idleAV * 0.5 and frictionTorque > idleTorque * 0.95)) then
    --if our friction is higher than the biggest torque we can output, the engine WILL lock up automatically
    --however, we need to communicate that with other subsystems to prevent issues, so in this case we ADDITIONALLY lock it up manually
  -- device:lockUp()
  end

  local compressionBrakeCoefAdjusted = device.throttle > 0 and 0 or device.compressionBrakeCoefDesired
  if compressionBrakeCoefAdjusted ~= device.compressionBrakeCoefActual then
    device.compressionBrakeCoefActual = compressionBrakeCoefAdjusted
    device:setEngineSoundParameter(device.engineSoundIDExhaust, "compression_brake_coef", device.compressionBrakeCoefActual, "exhaust")
  end

  local antiLagCoefAdjusted = device.antiLagCoefDesired
  if antiLagCoefAdjusted ~= device.antiLagCoefActual then
    device.antiLagCoefActual = antiLagCoefAdjusted
    device:setEngineSoundParameter(device.engineSoundIDExhaust, "triggerAntilag", device.antiLagCoefActual, "exhaust")
    device.turbocharger.setAntilagCoef(device.antiLagCoefActual)
  end

  device.exhaustFlowDelay:push(device.engineLoad)

  --push our summed fuels into the delay lines (shift fuel does not have any delay and therefore does not need a line)
  if device.shiftAfterFireFuel <= 0 then
    if device.instantAfterFireFuel > 0 then
      device.instantAfterFireFuelDelay:push(device.instantAfterFireFuel / dt)
    end
    if device.sustainedAfterFireFuel > 0 then
      device.sustainedAfterFireFuelDelay:push(device.sustainedAfterFireFuel / dt)
    end
  end

  if device.sustainedAfterFireTimer > 0 then
    device.sustainedAfterFireTimer = device.sustainedAfterFireTimer - dt
  elseif device.instantEngineLoad > 0 then
    device.sustainedAfterFireTimer = device.sustainedAfterFireTime
  end

  device.nitrousOxideTorque = 0 -- reset N2O torque
  device.engineVolumeCoef = 1 -- reset volume coef
  device.invBurnEfficiencyCoef = 1 -- reset burn efficiency coef

  device.turbocharger.updateGFX(dt)
  device.supercharger.updateGFX(dt)
  device.nitrousOxideInjection.updateGFX(dt)

  device.thermals.updateGFX(dt)

  -- Show flood percentage UI if flooded
  if device.floodLevel > 0 then
    -- Smooth the flood level value to prevent rapid fluctuations
    device.smoothedFloodLevel = device.smoothedFloodLevel or 0
    local smoothingFactor = 0.2  -- Higher = smoother but slower to respond
    device.smoothedFloodLevel = device.smoothedFloodLevel * (1 - smoothingFactor) + device.floodLevel * smoothingFactor
    
    -- Round to nearest percent for display
    local currPercent = floor(0.01 + device.smoothedFloodLevel * 100)
    local prevPercent = device.prevFloodPercent or 0
    
    -- Only update UI if percentage changed by at least 1% to prevent flickering
    if math.abs(currPercent - prevPercent) >= 1 or (currPercent == 0 and prevPercent > 0) then
      if device.floodLevel > 0.1 then
        -- Only show message if we're not cranking or if flood level is significant
        if device.starterEngagedCoef < 1 or device.floodLevel > 0.3 then
          guihooks.message("Engine flooding with fuel. \nFuel in Cylinders: " .. currPercent .. "%", 4, "vehicle.damage.flood")
        end
      else
        -- Reset flood level when below threshold and not cranking
        if device.starterEngagedCoef < 0.1 then
          device.floodLevel = 0
          device.smoothedFloodLevel = 0
        end
      end
      device.prevFloodPercent = currPercent
    end
  else
    -- Reset smoothed value when not flooded
    device.smoothedFloodLevel = 0
  end

  -- Update flood state
  device:updateFloodingState(dt)

  device.intakeAirDensityCoef = obj:getRelativeAirDensity()

  device:checkHydroLocking(dt)

  device.idleAVReadError = device.idleAVReadErrorSmoother:getUncapped(device.idleAVReadErrorRangeHalf - random(device.idleAVReadErrorRange), dt) * device.wearIdleAVReadErrorRangeCoef * device.damageIdleAVReadErrorRangeCoef
  device.idleAVStartOffset = device.idleAVStartOffsetSmoother:get(device.idleAV * device.idleStartCoef * device.starterEngagedCoef, dt)
  device.maxIdleAV = device.idleAV + device.idleAVReadErrorRangeHalf * device.wearIdleAVReadErrorRangeCoef * device.damageIdleAVReadErrorRangeCoef
  device.minIdleAV = device.idleAV - device.idleAVReadErrorRangeHalf * device.wearIdleAVReadErrorRangeCoef * device.damageIdleAVReadErrorRangeCoef

  device.spentEnergy = 0
  device.spentEnergyNitrousOxide = 0
  device.engineWorkPerUpdate = 0
  device.frictionLossPerUpdate = 0
  device.pumpingLossPerUpdate = 0

  device.instantAfterFireFuel = 0
  device.sustainedAfterFireFuel = 0
  device.shiftAfterFireFuel = 0
  device.continuousAfterFireFuel = 0
end

local function setTempRevLimiter(device, revLimiterAV, maxOvershootAV)
  device.tempRevLimiterAV = revLimiterAV
  device.tempRevLimiterMaxAVOvershoot = maxOvershootAV or device.tempRevLimiterAV * 0.01
  device.invTempRevLimiterRange = 1 / device.tempRevLimiterMaxAVOvershoot
  device.isTempRevLimiterActive = true
end

local function resetTempRevLimiter(device)
  device.tempRevLimiterAV = device.maxAV * 10
  device.tempRevLimiterMaxAVOvershoot = device.tempRevLimiterAV * 0.01
  device.invTempRevLimiterRange = 1 / device.tempRevLimiterMaxAVOvershoot
  device.isTempRevLimiterActive = false
  device:setExhaustGainMufflingOffsetRevLimiter(0, 0)
end

local function revLimiterDisabledMethod(device, engineAV, throttle, dt)
  return throttle
end

local function revLimiterSoftMethod(device, engineAV, throttle, dt)
  local limiterAV = min(device.revLimiterAV, device.tempRevLimiterAV)
  local correctedThrottle = -throttle * min(max(engineAV - limiterAV, 0), device.revLimiterMaxAVOvershoot) * device.invRevLimiterRange + throttle

  if device.isTempRevLimiterActive and correctedThrottle < throttle then
    device:setExhaustGainMufflingOffsetRevLimiter(-0.1, 2)
  end
  return correctedThrottle
end

local function revLimiterTimeMethod(device, engineAV, throttle, dt)
  local limiterAV = min(device.revLimiterAV, device.tempRevLimiterAV)
  if device.revLimiterActive then
    device.revLimiterActiveTimer = device.revLimiterActiveTimer - dt
    local revLimiterAVThreshold = min(limiterAV - device.revLimiterMaxAVDrop, limiterAV)
    --Deactivate the limiter once below the deactivation threshold
    device.revLimiterActive = device.revLimiterActiveTimer > 0 and engineAV > revLimiterAVThreshold
    device.revLimiterWasActiveTimer = 0
    return 0
  end

  if engineAV > limiterAV and not device.revLimiterActive then
    device.revLimiterActiveTimer = device.revLimiterCutTime
    device.revLimiterActive = true
    device.revLimiterWasActiveTimer = 0
    return 0
  end

  return throttle
end

local function revLimiterRPMDropMethod(device, engineAV, throttle, dt)
  local limiterAV = min(device.revLimiterAV, device.tempRevLimiterAV)
  if device.revLimiterActive or engineAV > limiterAV then
    --Deactivate the limiter once below the deactivation threshold
    local revLimiterAVThreshold = min(limiterAV - device.revLimiterAVDrop, limiterAV)
    device.revLimiterActive = engineAV > revLimiterAVThreshold
    device.revLimiterWasActiveTimer = 0
    return 0
  end

  return throttle
end

local function updateFixedStep(device, dt)
  -- Initialize carburetor if not already initialized
  if not device.carburetor then
    device.carburetor = carburetor.new(device)
    if device.carburetor then
      device.carburetor:initialize()
    end
  end

  --update idle throttle
  device.idleTimer = device.idleTimer - dt
  if device.idleTimer <= 0 then
    local idleTimeRandomCoef = linearScale(device.idleTimeRandomness, 0, 1, 1, randomGauss3() * 0.6666667)
    device.idleTimer = device.idleTimer + device.idleTime * idleTimeRandomCoef
    device.idleTime = device.idleTimer - dt
    local engineAV = device.outputAV1
    -- Calculate warm-up idle based on temperature
    local warmUpIdle = getWarmUpIdle(device)
    
    -- Adjust for carburetor state if initialized
    local carbIdleMultiplier = 1.0
    if device.carburetor then
      carbIdleMultiplier = device.carburetor:getIdleMultiplier()
    end
    warmUpIdle = warmUpIdle * carbIdleMultiplier
    
    -- Adjust for flooding state
    if device.isFlooded then
        local floodIdle = device.idleAV * 0.2  -- Reduce idle speed when flooded
        warmUpIdle = min(warmUpIdle, floodIdle)
    end
    
    -- Adjust for vapor lock state
    if device.isVaporLocked then
        -- Reduce idle speed and increase starter time when vapor locked
        local vaporLockIdle = device.idleAV * 0.2  -- Very low idle when vapor locked
        warmUpIdle = min(warmUpIdle, vaporLockIdle)
        
        -- Increase starter time significantly when vapor locked or flooded
        if device.starterEngagedCoef == 1 then
            local starterTimeMultiplier = 1
            if device.isVaporLocked then
                starterTimeMultiplier = starterTimeMultiplier * (1 + device.vaporLockLevel * 2)
            end
            if device.isFlooded then
                starterTimeMultiplier = starterTimeMultiplier * 1.5 -- Add 50% more starter time when flooded
            end
            device.starterThrottleKillTimer = device.starterThrottleKillTimer * starterTimeMultiplier
        end
    end
    
    local highIdle = device.idleAV + warmUpIdle
    local idleAV = max(highIdle, device.idleAVOverwrite)
    local maxIdleThrottle = min(max(device.maxIdleThrottle, device.maxIdleThrottleOverwrite), 1)
    local idleAVError = max(idleAV - engineAV + device.idleAVReadError + device.idleAVStartOffset, 0)
    device.idleThrottleTarget = min(idleAVError * device.idleControllerP, maxIdleThrottle)

  --print(device.idleThrottle)
  end
  device.idleThrottle = device.idleThrottleSmoother:get(device.idleThrottleTarget, dt)

  device.forcedInductionCoef = 1
  device.turbocharger.updateFixedStep(dt)
  device.supercharger.updateFixedStep(dt)
end

--velocity update is always nopped for engines

-- Engine flooding constants
local engineFloodValues = {
    threshold = 10.0, -- Increased to 10 seconds to make flooding less quick
    torqueMultiplier = 0.7, -- Increased to 70% of normal torque during flooding
    rpmLimit = 0.4, -- Increased RPM limit to 10% during flooding
    throttleReduction = 0.8, -- Reduced throttle reduction to 80% per flood level
    roughRunningIntensity = 0.5, -- Base rough running intensity
    roughRunningVariation = 0.2, -- Variation in rough running
    rpmFluctuation = 0.4, -- RPM fluctuation multiplier
    torqueTransitionTime = 0.1, -- Time for torque transitions
    idlePenalty = 0.9, -- Idle RPM penalty during flooding
    starterRPMLimit = 0.9 -- Starter RPM limit multiplier
}

-- Reset flood state variables
local function resetFloodState(device)
    device.floodTimer = 0
    device.floodLevel = 0
    device.isFlooded = false
    device.roughRunning = false
    device.roughRunningIntensity = 0
    device.starterRoughness = 0
end

local function updateTorque(device, dt)
  
  local engineAV = device.outputAV1

  local throttle = (electrics.values[device.electricsThrottleName] or 0) * (electrics.values[device.electricsThrottleFactorName] or device.throttleFactor)

  --don't include idle throttle as otherwise idle affects the turbo wastegate, do include it though if we have a raised idle throttle (eg semi truck hidh idle)
  device.requestedThrottle = max(throttle, device.idleAVOverwrite > 0 and device.idleThrottle or 0)

  throttle = min(max(max(device.idleThrottle, throttle) * (device.starterThrottleKillCoef+(1-device.starterIgnitionErrorCoef)*device.inertia*0.1) * device.ignitionCoef, 0), 1)

  throttle = device:applyRevLimiter(engineAV, throttle, dt)

  --smooth our actual throttle value to simulate various effects in a real engine that do not allow immediate throttle changes
  throttle = device.throttleSmoother:getUncapped(throttle, dt) * 0.5

  local finalFriction = device.friction * device.wearFrictionCoef * device.damageFrictionCoef
  local finalDynamicFriction = device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef

  local tableRPM = floor(engineAV * avToRPM) or 0
  local torque = (device.torqueCurve[tableRPM] or 0) * device.intakeAirDensityCoef
  local maxCurrentTorque = torque - finalFriction - (finalDynamicFriction * engineAV)
  --blend pure throttle with the constant power map
  local throttleMap = smoothmin(max(throttle + throttle * device.maxPowerThrottleMap / (torque * device.forcedInductionCoef * engineAV + 0.001) * (1 - throttle), 0), 1, (1 - throttle) * 0.8) --0.8 can be tweaked to reduce he peakiness of the throttlemap adjusted torque curve

  local ignitionCut = device.ignitionCutTime > 0
  torque = ((torque * device.forcedInductionCoef * throttleMap) + device.nitrousOxideTorque) * device.outputTorqueState * (ignitionCut and 0 or 1) * device.slowIgnitionErrorCoef * device.fastIgnitionErrorCoef * device.starterIgnitionErrorCoef
  torque = min(torque, device.maxTorqueLimit) --limit output torque to a specified max, math.huge by default

  local lastInstantEngineLoad = device.instantEngineLoad
  local instantLoad = min(max(torque / ((maxCurrentTorque + 1e-30) * device.outputTorqueState * device.forcedInductionCoef), 0), 1)
  device.instantEngineLoad = instantLoad
  device.engineLoad = device.loadSmoother:getCapped(device.instantEngineLoad, dt)
  local normalizedEngineAV = clamp(engineAV / device.maxAV, 0, 1)
  local revLimiterActive = device.revLimiterWasActiveTimer < 0.1
  device.exhaustFlowCoef = revLimiterActive and (device.revLimiterActiveMaxExhaustFlowCoef * normalizedEngineAV) or device.engineLoad

  local absEngineAV = abs(engineAV)
  local dtT = dt * torque
  local dtTNitrousOxide = dt * device.nitrousOxideTorque

  local burnEnergy = dtT * (dtT * device.halfInvEngInertia + engineAV)
  local burnEnergyNitrousOxide = dtTNitrousOxide * (dtTNitrousOxide * device.halfInvEngInertia + engineAV)
  device.engineWorkPerUpdate = device.engineWorkPerUpdate + burnEnergy
  device.frictionLossPerUpdate = device.frictionLossPerUpdate + finalFriction * absEngineAV * dt
  device.pumpingLossPerUpdate = device.pumpingLossPerUpdate + finalDynamicFriction * engineAV * engineAV * dt
  local invBurnEfficiency = device.invBurnEfficiencyTable[floor(device.instantEngineLoad * 100)] * device.invBurnEfficiencyCoef
  device.spentEnergy = device.spentEnergy + burnEnergy * invBurnEfficiency
  device.spentEnergyNitrousOxide = device.spentEnergyNitrousOxide + burnEnergyNitrousOxide * invBurnEfficiency

  local compressionBrakeTorque = (device.compressionBrakeCurve[tableRPM] or 0) * device.compressionBrakeCoefActual
  --todo check why this is not included in thermals
  local engineBrakeTorque = device.engineBrakeTorque * (1 - min(instantLoad + device.antiLagCoefActual, 1))
  local frictionTorque = finalFriction + finalDynamicFriction * absEngineAV + engineBrakeTorque
  --friction torque is limited for stability
  frictionTorque = min(frictionTorque, absEngineAV * device.inertia * 300) * sign(engineAV)

  -- Initialize starter variables if not set
  device.starterTorqueMultiplier = device.starterTorqueMultiplier or 0.1  -- Further reduced base multiplier for better control
  device.starterStress = device.starterStress or 0
  device.starterKillTimer = device.starterKillTimer or 0
  device.starterEngagedTime = device.starterEngagedTime or 0
  device.floodLevel = device.floodLevel or 0
  
  -- Override starterMaxAV to allow higher cranking speeds if not already set
  if not device._starterMaxAVOverridden then
    -- Calculate starterMaxAV in RPM first, then convert to AV
    local targetRPM = (device.idleRPM or 100) * 0.4  -- Reduced to 40% of idle RPM
    device.starterMaxAV = targetRPM * rpmToAV  -- Convert RPM to AV
    device.invStarterMaxAV = 1 / device.starterMaxAV
    device._starterMaxAVOverridden = true
  end
  
  -- Initialize starter torque to zero by default
  local starterTorque = 0
  
  -- Only process starter logic if actually cranking
  if device.starterEngagedCoef > 0 then
    -- Calculate base starter torque with multiplier
    starterTorque = device.starterTorque * device.starterTorqueMultiplier
    
    -- Update cranking time and stress
    device.starterEngagedTime = device.starterEngagedTime + dt
    
    -- Handle flooded state
    if device.isFlooded then
      -- Apply torque boost and stress effects
      device.starterStress = min(device.starterStress + (dt * 0.1), 1.0)
      
      -- Enhanced torque reduction based on stress and flood level
      local reduction = (device.starterStress * 0.1) + (device.floodLevel * 0.05)  -- Reduced both factors
      starterTorque = starterTorque * (1 - min(reduction, 0.3))  -- Reduced max reduction
      
      -- More noticeable struggles
      local struggleSeverity = device.starterStress * 0.15
      starterTorque = starterTorque * (1 - struggleSeverity)
      if struggleSeverity > 0.3 then
        guihooks.message("Starter struggling hard!", 2.0, "vehicle.damage")
      end
      
      -- Calculate torque bounds for stressed/flooded state
      local baseTorque = device.starterTorque * device.starterTorqueMultiplier
      minStarterTorque = baseTorque * 0.2  -- Reduced for more controlled stressed cranking
      maxStarterTorque = baseTorque * 0.4  -- Reduced for more controlled stressed cranking
      
      -- Apply torque bounds
      starterTorque = clamp(starterTorque, minStarterTorque, maxStarterTorque)
      
      -- Auto-shutoff after extended cranking (60-90 seconds based on flood level)
      device.starterKillTimer = device.starterKillTimer + dt
      if device.starterKillTimer > (60 + device.floodLevel * 30) then
        device.starterEngagedCoef = 0
        device.starterKillTimer = 0
        device.starterDisabled = true
        device.starterThrottleKillTimer = 0  -- Reset throttle kill timer when auto-shutoff occurs
        guihooks.message("Starter disabled - let it cool down", 3, "vehicle.damage")
      end
    else
      -- Normal operation - faster stress recovery
      device.starterStress = max(0, device.starterStress - (dt * 0.3))  -- Increased recovery rate
      device.starterKillTimer = 0
      device.starterDisabled = false
    end
    
    -- Adjust torque bounds for more controlled cranking
    local baseTorque = device.starterTorque * device.starterTorqueMultiplier
    minStarterTorque = baseTorque * 0.008  -- Reduced for even more controlled cranking
    maxStarterTorque = baseTorque * 0.015  -- Reduced for even more controlled cranking
    
    -- Apply torque bounds
    starterTorque = clamp(starterTorque, minStarterTorque, maxStarterTorque)
    
  else
    -- Reset timers when not cranking
    device.starterEngagedTime = 0
    device.starterKillTimer = 0
    device.starterStress = max(0, device.starterStress - (dt * 0.2))  -- Increased recovery rate to 0.2
    device.starterDisabled = false
  end
  
  -- Apply starter stress to final torque (less severe penalty)
  starterTorque = starterTorque * (1 - (1 - device.starterStress) * 0.3)  -- Reduced penalty from 100% to 30%
  -- Only disengage starter when engine is running above idle
  if device.outputAV1 > device.idleAV * 1.1 then
    device.starterEngagedCoef = 0  -- Instant disengagement when engine starts
    -- Reset stress and disable timer
    device.starterStress = 0
    device.starterKillTimer = 0
    device.starterDisabled = false
  end
  
  -- Initialize or update engine coast down state
  if device.ignitionCoef <= 0 and device.lastIgnitionCoef > 0 then
    -- Just turned off - store current RPM
    device.coastDownRPM = device.outputAV1 * avToRPM
    device.coastDownTime = 0
  end
  device.lastIgnitionCoef = device.ignitionCoef
  
  -- Adjust starter torque based on conditions and flooding state
  local starterEfficiency = 1
  local tempFactor = device.thermals.engineBlockTemperature - 273.15  -- Convert K to °C
  -- Calculate starter efficiency based on temperature and condition
  if tempFactor < 30 then  -- Start adjusting efficiency below 30°C
      tempFactor = (tempFactor + 30) / 20  -- Normalize from -20°C to 10°C
      starterEfficiency = tempFactor * -0.1  -- 70% efficiency (hopefully)
    end
  
  -- Apply starter stress penalty (overheating) - less severe
  if device.starterStress then
    starterEfficiency = starterEfficiency * (1 - device.starterStress * 0.2)  -- Reduced from 0.3 to 0.2
  end
  
  -- Base torque values
  if device.starterEngagedCoef == 1 then
    if device.isFlooded then
      -- When flooded, use dynamic torque based on flood level and starter stress
      local floodFactor = 1 - (device.floodLevel * 0.5)  -- Reduced flood penalty from 70% to 50%
      local stressFactor = 1 - (device.starterStress * 0.1)  -- Reduced stress penalty from 15% to 10%
      
      -- Calculate torque with reduced variations
      minStarterTorque = device.starterTorque * device.starterTorqueMultiplier * 0.8 * floodFactor * stressFactor * starterEfficiency
      maxStarterTorque = device.starterTorque * device.starterTorqueMultiplier * 1.4 * floodFactor * stressFactor * starterEfficiency
      starterTorque = starterTorque * 0.55 * floodFactor * stressFactor
    else
      -- Normal operation - consistent torque values with better base torque
      minStarterTorque = device.starterTorque * 0.7 * starterEfficiency  -- Increased from 0.6
      maxStarterTorque = device.starterTorque * 0.9 * starterEfficiency  -- Increased from 0.8
      starterTorque = starterTorque * 0.9  -- Increased from 0.8
    
    -- Apply torque bounds
    starterTorque = clamp(starterTorque, minStarterTorque, maxStarterTorque)
  end

  -- Add starter motor sound effect based on load
  if device.starterSound then
    local loadPitch = 0.8 + (1 - (starterTorque / maxStarterTorque)) * 0.4
    device.starterSound:setPitch(loadPitch)
    
    -- Add subtle rumble when struggling
    if device.isFlooded then
      local rumble = math.sin(device.floodTimer * 10) * 0.05 * device.floodLevel
      device.starterSound:setPitch(loadPitch + rumble)
      end
    end
  end

  --iterate over all connected clutches and sum their torqueDiff to know the final torque load on the engine
  local torqueDiffSum = 0
  for i = 1, device.activeOutputPortCount do
    local outputPort = device.activeOutputPorts[i]
    torqueDiffSum = torqueDiffSum + device.clutchChildren[outputPort].torqueDiff
  end
  -- Calculate base angular velocity
  local outputAV = (engineAV + dt * (torque - torqueDiffSum - frictionTorque - compressionBrakeTorque + starterTorque) * device.invEngInertia) * device.outputAVState
  
  -- Calculate maximum allowed RPM for the starter
  local maxStarterRPM = device.idleAV * math.random(2.0, 2.4)  -- Base max is 1/3 of idle speed  
  -- Apply temperature-based RPM reduction (colder = harder to turn over)
  if device.thermals and device.thermals.engineBlockTemperature then
    local tempC = device.thermals.engineBlockTemperature - 273.15  -- Convert K to °C
    if tempC < 10 then
      -- Reduce max RPM in cold conditions (down to 70% at -20°C)
      local tempFactor = clamp(0.7 + (tempC + 20) * 0.01, 0.7, 1.0)
      maxStarterRPM = maxStarterRPM * tempFactor
    end
  end
  
  -- Handle starter engagement
  if device.starterEngagedCoef > 0 then
    -- Initialize ignition error timers and chance values if not set
    if not device.slowIgnitionErrorTimer then
      device.slowIgnitionErrorTimer = 0
    end
    if not device.fastIgnitionErrorTimer then
      device.fastIgnitionErrorTimer = 0
    end
    if not device.slowIgnitionErrorChance then
      device.slowIgnitionErrorChance = 0
    end
    if not device.fastIgnitionErrorChance then
      device.fastIgnitionErrorChance = 0
    end
    
    -- Apply flood-based RPM reduction
    if device.isFlooded then
      -- Reduce max RPM based on flood level (down to 50% when fully flooded)
      local floodFactor = 1.0 - (device.floodLevel * 0.5)
     maxStarterRPM = maxStarterRPM * floodFactor
      
      -- Add consistent RPM reduction based on flood level and starter stress
      local struggle = 0.95 + (device.starterStress or 0) * 0.05
      outputAV = min(outputAV * struggle, maxStarterRPM)
      
      -- Apply consistent RPM reduction when severely flooded
      if device.floodLevel > 0.7 then
        outputAV = outputAV * 0.75
      end
      
      -- Prevent stalling at full throttle
      if device.throttle > 0.2 then
        outputAV = max(outputAV, device.idleAV * 0.2)  -- Maintain at least 20% of idle speed
      end
      
      -- Add misfire effect during cranking if flooded
    if device.floodLevel > 0.5 then
        device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer + dt * 0.3
        device.fastIgnitionErrorTimer = device.fastIgnitionErrorTimer + dt * 0.3
        outputAV = outputAV * 0.5
    end
    end
    
    -- Final RPM limiting
    outputAV = min(outputAV, maxStarterRPM)
    
    -- Update starter stress based on load (higher RPM = more stress)
    if device.starterStress then
      local rpmRatio = outputAV / maxStarterRPM
      local stressGain = dt * 0.02 * (0.5 + rpmRatio * 0.5)  -- Reduced stress gain
      device.starterStress = min(1.0, (device.starterStress or 0) + stressGain)
    end
  else
    -- Reset ignition error chance when starter is disengaged
    device.starterIgnitionErrorChance = 0
    
    -- Cool down starter when not in use
    if device.starterStress then
      device.starterStress = max(0, (device.starterStress or 0) - (dt * 0.02))
    end
  end
  
  -- Handle engine coast down when ignition is off
  if device.ignitionCoef <= 0 and outputAV > 1.1 then
    -- Calculate RPM values
    local rpm = outputAV * avToRPM
    local idleRPM = device.idleAV * avToRPM
    
    -- Calculate deceleration rate (slower at lower RPMs)
    local rpmRatio = rpm / (idleRPM * 2)  -- Normalize to idle RPM
    local decelerationRate = 0.2 * rpmRatio * rpmRatio  -- Quadratic falloff
    
    -- Apply consistent deceleration
    local decel = decelerationRate * dt * 0.95
    outputAV = max(0, outputAV - decel)
    
    -- Ensure clean stop
    if outputAV < 0.1 then 
      outputAV = 0 
    end
  end

  --set all output torques and AVs to the newly calculated values
  for i = 1, device.activeOutputPortCount do
    local outputPort = device.activeOutputPorts[i]
    device[device.outputTorqueNames[outputPort]] = torqueDiffSum
    device[device.outputAVNames[outputPort]] = outputAV
  end
  device.throttle = throttle
  device.combustionTorque = torque - frictionTorque
  device.frictionTorque = frictionTorque

  local inertialTorque = (device.outputAV1 - device.lastOutputAV1) * device.inertia / dt
  obj:applyTorqueAxisCouple(inertialTorque, device.torqueReactionNodes[1], device.torqueReactionNodes[2], device.torqueReactionNodes[3])
  device.lastOutputAV1 = device.outputAV1

  local dLoad = min((device.instantEngineLoad - lastInstantEngineLoad) / dt, 0)
  local instantAfterFire = engineAV > device.idleAV * 2 and max(device.instantAfterFireCoef * -dLoad * lastInstantEngineLoad * absEngineAV, 0) or 0
  local sustainedAfterFire = (device.instantEngineLoad <= 0 and device.sustainedAfterFireTimer > 0) and max(engineAV * device.sustainedAfterFireCoef, 0) or 0

  device.instantAfterFireFuel = device.instantAfterFireFuel + instantAfterFire
  device.sustainedAfterFireFuel = device.sustainedAfterFireFuel + sustainedAfterFire
  device.shiftAfterFireFuel = device.shiftAfterFireFuel + instantAfterFire * (ignitionCut and 1 or 0)

  device.lastOutputTorque = torque
  device.ignitionCutTime = max(device.ignitionCutTime - dt, 0)

  device.fixedStepTimer = device.fixedStepTimer + dt
  if device.fixedStepTimer >= device.fixedStepTime then
    device:updateFixedStep(device.fixedStepTimer)
    device.fixedStepTimer = device.fixedStepTimer - device.fixedStepTime
  end
end

local function selectUpdates(device)
  device.velocityUpdate = nop
  device.torqueUpdate = updateTorque
end

local function applyDeformGroupDamage(device, damageAmount, groupType)
  if groupType == "main" then
    device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
    device.damageDynamicFrictionCoef = device.damageDynamicFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
    device.damageIdleAVReadErrorRangeCoef = device.damageIdleAVReadErrorRangeCoef + linearScale(damageAmount, 0, 0.01, 0, 0.5)
    device.fastIgnitionErrorChance = min(device.fastIgnitionErrorChance + linearScale(damageAmount, 0, 0.01, 0, 0.05))
    device.slowIgnitionErrorChance = min(device.slowIgnitionErrorChance + linearScale(damageAmount, 0, 0.01, 0, 0.05))
    damageTracker.setDamage("engine", "impactDamage", true, true)
  elseif groupType == "radiator" and device.thermals.applyDeformGroupDamageRadiator then
    device.thermals.applyDeformGroupDamageRadiator(damageAmount)
  elseif groupType == "oilPan" and device.thermals.applyDeformGroupDamageOilpan then
    device.thermals.applyDeformGroupDamageOilpan(damageAmount)
  elseif groupType == "oilRadiator" and device.thermals.applyDeformGroupDamageOilRadiator then
    device.thermals.applyDeformGroupDamageOilRadiator(damageAmount)
  elseif groupType == "turbo" and device.turbocharger.applyDeformGroupDamage then
    device.turbocharger.applyDeformGroupDamage(damageAmount)
  elseif groupType == "supercharger" and device.supercharger.applyDeformGroupDamage then
    device.supercharger.applyDeformGroupDamage(damageAmount)
  end
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  if not subSystem then
    device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 1.0)
    device.wearDynamicFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 1.5)
    device.wearIdleAVReadErrorRangeCoef = linearScale(odometer, 30000000, 500000000, 1, 10)
    local integrityState = integrity
    if type(integrity) == "number" then
      local integrityValue = integrity
      integrityState = {
        damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 1.0),
        damageDynamicFrictionCoef = linearScale(integrityValue, 1, 0, 1, 1.5),
        damageIdleAVReadErrorRangeCoef = linearScale(integrityValue, 1, 0, 1, 30),
        fastIgnitionErrorChance = linearScale(integrityValue, 1, 0, 0, 0.4),
        slowIgnitionErrorChance = linearScale(integrityValue, 1, 0, 0, 0.4)
      }
    end

    device.damageFrictionCoef = integrityState.damageFrictionCoef or 1
    device.damageDynamicFrictionCoef = integrityState.damageDynamicFrictionCoef or 1
    device.damageIdleAVReadErrorRangeCoef = integrityState.damageIdleAVReadErrorRangeCoef or 1
    device.fastIgnitionErrorChance = integrityState.fastIgnitionErrorChance
    device.slowIgnitionErrorChance = integrityState.slowIgnitionErrorChance

    device.thermals.setPartConditionThermals(odometer, integrityState.thermals or {}, visual)

    if integrityState.isBroken then
      device:onBreak()
    end
  elseif subSystem == "radiator" then
    device.thermals.setPartConditionRadiator(odometer, integrity, visual)
  elseif subSystem == "exhaust" then
    device.thermals.setPartConditionExhaust(odometer, integrity, visual)
  elseif subSystem == "turbocharger" then
    device.turbocharger.setPartCondition(odometer, integrity, visual)
  -- elseif subSystem == "supercharger" then
  --   device.supercharger.setPartCondition(odometer, integrity, visual)
  end
end

local function getPartCondition(device, subSystem)
  if not subSystem then
    local integrityState = {
      damageFrictionCoef = device.damageFrictionCoef,
      damageDynamicFrictionCoef = device.damageDynamicFrictionCoef,
      damageIdleAVReadErrorRangeCoef = device.damageIdleAVReadErrorRangeCoef,
      fastIgnitionErrorChance = device.fastIgnitionErrorChance,
      slowIgnitionErrorChance = device.slowIgnitionErrorChance,
      isBroken = device.isBroken
    }

    local frictionIntegrityValue = linearScale(device.damageFrictionCoef, 1, 5, 1, 0)
    local dynamicFrictionIntegrityValue = linearScale(device.damageDynamicFrictionCoef, 1, 5, 1, 0)
    local idleAVReadErrorRangeIntegrityValue = linearScale(device.damageIdleAVReadErrorRangeCoef, 1, 50, 1, 0)
    local slowIgnitionErrorIntegrityValue = linearScale(device.slowIgnitionErrorChance, 0, 0.4, 1, 0)
    local fastIgnitionErrorIntegrityValue = linearScale(device.fastIgnitionErrorChance, 0, 0.4, 1, 0)

    local integrityValueThermals, partConditionThermals = device.thermals.getPartConditionThermals()
    integrityState.thermals = partConditionThermals

    local integrityValue = min(frictionIntegrityValue, dynamicFrictionIntegrityValue, idleAVReadErrorRangeIntegrityValue, slowIgnitionErrorIntegrityValue, fastIgnitionErrorIntegrityValue, integrityValueThermals)
    if device.isBroken then
      integrityValue = 0
    end
    return integrityValue, integrityState
  elseif subSystem == "exhaust" then
    local integrityValue, integrityState = device.thermals.getPartConditionExhaust()
    return integrityValue, integrityState
  elseif subSystem == "radiator" then
    local integrityValue, integrityState = device.thermals.getPartConditionRadiator()
    return integrityValue, integrityState
  elseif subSystem == "turbocharger" then
    local integrityValue, integrityState = device.turbocharger.getPartCondition()
    return integrityValue, integrityState
  -- elseif subSystem == "supercharger" then
  --   local integrityValue, integrityState = device.supercharger.getPartCondition()
  --   return integrityValue, integrityState
  end
end

local function validate(device)

  -- Initialize carburetor after thermals
  device.carburetor = carburetor.new(device)
  if device.carburetor then
    device.carburetor:initialize()
  end

  device.clutchChildren = {}
  if device.children and #device.children > 0 then
    for _, child in ipairs(device.children) do
      if child.deviceCategories.clutchlike then
        device.clutchChildren[child.inputIndex] = child
        device.inertia = device.inertia + (child.additionalEngineInertia or 0)
      else
        log("E", "combustionEngine.validate", "Found a non clutchlike device as child of a combustion engine!")
        log("E", "combustionEngine.validate", "Child data:")
        log("E", "combustionEngine.validate", powertrain.dumpsDeviceData(child))
        return false
      end
    end
    device.invEngInertia = 1 / device.inertia
    device.halfInvEngInertia = device.invEngInertia * 0.5
  end
  device.initialInertia = device.inertia

  table.insert(
    powertrain.engineData,
    {
      maxRPM = device.maxRPM,
      maxSoundRPM = device.hasRevLimiter and device.maxRPM or device.maxAvailableRPM,
      torqueReactionNodes = device.torqueReactionNodes
    }
  )

  device.activeOutputPorts = {}
  local spawnWithEngineRunning = device.spawnVehicleIgnitionLevel > 2
  local spawnAV = spawnWithEngineRunning and device.idleAV or 0

  --iterate over the advertised output ports
  for i = 1, device.numberOfOutputPorts do
    --check if we have a child that wants to connect to that port
    local childForPort
    for _, child in ipairs(device.children or {}) do
      if i == child.inputIndex then
        childForPort = child
        break
      end
    end
    --if we found one OR if we look at the port 1 (which always needs to exist for other systems), configure the data for this port
    if childForPort or i == 1 then
      table.insert(device.activeOutputPorts, i)
      --cache the required output torque and AV property names for fast access
      device.outputTorqueNames[i] = "outputTorque" .. tostring(i)
      device.outputAVNames[i] = "outputAV" .. tostring(i)
      device[device.outputTorqueNames[i]] = 0
      device[device.outputAVNames[i]] = spawnAV
    else
      --if no child or port 1, disable this port
      device.outputPorts[i] = false
    end
  end
  --we always need at least a dummy clutch child on output 1 for other stuff to work
  device.clutchChildren[1] = device.clutchChildren[1] or {torqueDiff = 0}

  device.outputRPM = device.outputAV1 * avToRPM
  device.lastOutputAV1 = device.outputAV1
  device.activeOutputPortCount = #device.activeOutputPorts

  return true
end

local function activateStarter(device)
    device.ignitionCoef = 1
    if device.starterEngagedCoef ~= 1 then
        -- Set choke based on temperature
        if device.thermals.engineBlockTemperature < 0 then
            device.carburetor:setChoke(true)
        end
        
        -- Initialize flooding state if not already
        if not device.floodTimer then
            device.floodTimer = 0
            device.floodLevel = 0
            device.isFlooded = false
        end
    end

    device.starterThrottleKillCoef = 0
    local coldBlockStartTimeCoef = device.requiredEnergyType == "diesel" and 31 or 15
    if device.lastStarterThrottleKillTimerEnd and device.lastStarterThrottleKillTimerEnd > 0.2 then
      device.starterThrottleKillTimer = device.lastStarterThrottleKillTimerEnd or device.starterThrottleKillTime
    else
      -- Adjust starter time based on temperature and flooding state
    local baseTime = device.starterThrottleKillTime * linearScale(device.thermals.engineBlockTemperature, -35, 20, coldBlockStartTimeCoef, 1)
    if device.isFlooded then
        -- Increase starter time when flooded
        baseTime = baseTime * (1 + device.floodLevel)
    end
    device.starterThrottleKillTimer = baseTime
    end

    device.starterThrottleKillTimerStart = device.starterThrottleKillTimer
    device.starterEngagedCoef = 1

    -- Only play starter sounds
    obj:cutSFX(device.engineMiscSounds.starterSoundEngine)
    obj:playSFX(device.engineMiscSounds.starterSoundEngine)

    if device.engineMiscSounds.starterSoundExhaust then
      obj:cutSFX(device.engineMiscSounds.starterSoundExhaust)
      obj:playSFX(device.engineMiscSounds.starterSoundExhaust)
    end

   -- device.engineMiscSounds.loopTimer = device.engineMiscSounds.loopTime
end

local function cutIgnition(device, time)
  device.ignitionCutTime = time
end

local function deactivateStarter(device)
  -- Check if engine speed is above idle speed to determine successful start
  local didStart = false
  if device.starterThrottleKillTimer <= 0 then
    device.starterThrottleKillCoef = 1
    didStart = true
  end
  device.starterThrottleKillTimer = 0
  device.starterEngagedCoef = 0
  if didStart then
    obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
    end
  else
    obj:cutSFX(device.engineMiscSounds.starterSoundEngine)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:cutSFX(device.engineMiscSounds.starterSoundExhaust)
    end
  end
end

local function setIgnition(device, value)
  device.ignitionCoef = value > 0 and 1 or 0
  if value == 0 then
    device.starterThrottleKillTimer = 0
    device.starterEngagedCoef = 0
    if device.outputAV1 > device.idleAV * 0.8 then
      device.shutOffSoundRequested = true
    end
  end
end

local function setCompressionBrakeCoef(device, coef)
  device.compressionBrakeCoefDesired = clamp(coef, 0, 1)
end

local function setAntilagCoef(device, coef)
  device.antiLagCoefDesired = clamp(coef, 0, 1)
end

local function onBreak(device)
  device:lockUp()
end

local function beamBroke(device, id)
  device.thermals.beamBroke(id)
end

local function registerStorage(device, storageName)
  local storage = energyStorage.getStorage(storageName)
  if not storage then
    return
  end
  if storage.type == "n2oTank" then
    device.nitrousOxideInjection.registerStorage(storageName)
  elseif storage.type == "electricBattery" then
    device.starterBattery = storage
  elseif storage.energyType == device.requiredEnergyType then
    device.storageWithEnergyCounter = device.storageWithEnergyCounter + 1
    table.insert(device.registeredEnergyStorages, storageName)
    device.previousEnergyLevels[storageName] = storage.storedEnergy
    device:updateEnergyStorageRatios()
    device:updateFuelUsage()
  end
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
    maxCumulativeGearRatio = child.maxCumulativeGearRatio
  end

  device.cumulativeInertia = outputInertia
  device.cumulativeGearRatio = cumulativeGearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio
end

local function initEngineSound(device, soundID, samplePath, engineNodeIDs, offLoadGain, onLoadGain, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].blendFile = samplePath

  device:setSoundLocation("engine", "Engine: " .. device.soundConfiguration.engine.blendFile, engineNodeIDs)

  obj:queueGameEngineLua(string.format("core_sounds.initEngineSound(%d,%d,%q,%s,%f,%f)", objectId, soundID, samplePath, serialize(engineNodeIDs), offLoadGain, onLoadGain))
end

local function initExhaustSound(device, soundID, samplePath, exhaustNodeIDPairs, offLoadGain, onLoadGain, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].blendFile = samplePath

  local nodeCids = {}
  for _, nodePair in pairs(exhaustNodeIDPairs) do
    table.insert(nodeCids, nodePair[2])
  end
  device:setSoundLocation("exhaust", "Exhaust: " .. device.soundConfiguration.exhaust.blendFile, nodeCids)

  obj:queueGameEngineLua(string.format("core_sounds.initExhaustSound(%d,%d,%q,%s,%f,%f)", objectId, soundID, samplePath, serialize(exhaustNodeIDPairs), offLoadGain, onLoadGain))
end

local function setExhaustSoundNodes(device, soundID, exhaustNodeIDPairs)
  local nodeCids = {}
  for _, nodePair in pairs(exhaustNodeIDPairs) do
    table.insert(nodeCids, nodePair[2])
  end
  device:setSoundLocation("exhaust", "Exhaust: " .. device.soundConfiguration.exhaust.blendFile, nodeCids)

  obj:queueGameEngineLua(string.format("core_sounds.setExhaustSoundNodes(%d,%d,%s)", objectId, soundID, serialize(exhaustNodeIDPairs)))
end

--this does not update aggregate parameters like main_gain or _muffled, use the list API for these
--it also does not update starter sound params
local function setEngineSoundParameter(device, soundID, paramName, paramValue, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].params = device.soundConfiguration[reference].params or {}
  device.soundConfiguration[reference].soundID = soundID
  local params = device.soundConfiguration[reference].params
  params[paramName] = paramValue
  obj:queueGameEngineLua(string.format("core_sounds.setEngineSoundParameter(%d,%d,%q,%f)", objectId, soundID, paramName, paramValue))
end

local function setEngineSoundParameterList(device, soundID, params, reference)
  params.main_gain = params.base_gain + params.gainOffset + params.gainOffsetRevLimiter
  params.muffled = params.base_muffled + params.mufflingOffset + params.mufflingOffsetRevLimiter

  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].params = tableMergeRecursive(device.soundConfiguration[reference].params or {}, params)
  device.soundConfiguration[reference].soundID = soundID
  obj:queueGameEngineLua(string.format("core_sounds.setEngineSoundParameterList(%d,%d,%s)", objectId, soundID, serialize(params)))

  --print(reference)
  --print(params.eq_e_gain)
  if reference == "engine" then
    if device.engineMiscSounds.starterSoundEngine then
      obj:setVolumePitchCT(device.engineMiscSounds.starterSoundEngine, device.engineMiscSounds.starterVolume, 1, params.main_gain, 0)
    end
    if device.engineMiscSounds.shutOffSoundEngine then
      obj:setVolumePitchCT(device.engineMiscSounds.shutOffSoundEngine, device.engineMiscSounds.shutOffVolumeEngine, 1, params.main_gain, 0)
    end
  elseif reference == "exhaust" then
    if device.engineMiscSounds.starterSoundExhaust then
      obj:setVolumePitchCT(device.engineMiscSounds.starterSoundExhaust, device.engineMiscSounds.starterVolumeExhaust, 1, params.main_gain, 0)
    end
    if device.engineMiscSounds.shutOffSoundExhaust then
      obj:setVolumePitchCT(device.engineMiscSounds.shutOffSoundExhaust, device.engineMiscSounds.shutOffVolumeExhaust, 1, params.main_gain, 0)
    end
  end
end

local function exhaustEndNodesChanged(device, endNodes)
  if device.engineSoundIDExhaust then
    local endNodeIDPairs
    local maxExhaustAudioOpennessCoef = 0
    local maxExhaustAudioGain
    if endNodes and #endNodes > 0 then
      endNodeIDPairs = {}
      for _, v in pairs(endNodes) do
        maxExhaustAudioOpennessCoef = min(max(maxExhaustAudioOpennessCoef, v.exhaustAudioOpennessCoef), 1)
        maxExhaustAudioGain = maxExhaustAudioGain and max(maxExhaustAudioGain, v.exhaustAudioGainChange) or v.exhaustAudioGainChange
        table.insert(endNodeIDPairs, {v.start, v.finish})
      end
    else
      endNodeIDPairs = {{device.engineNodeID, device.engineNodeID}}
      maxExhaustAudioGain = 0
    end
    device:setExhaustSoundNodes(device.engineSoundIDExhaust, endNodeIDPairs)

    local params = {
      base_muffled = device.exhaustAudioMufflingMinCoef + device.exhaustAudioMufflingCoefRange * (1 - maxExhaustAudioOpennessCoef),
      base_gain = device.exhaustMainGain + maxExhaustAudioGain,
      gainOffset = 0,
      mufflingOffset = 0,
      mufflingOffsetRevLimiter = 0,
      gainOffsetRevLimiter = 0
    }
    device:setEngineSoundParameterList(device.engineSoundIDExhaust, params, "exhaust")
  end
end

local function setSoundLocation(device, soundType, displayText, nodeCids)
  device.soundLocations[soundType] = {text = displayText or "", nodes = nodeCids}
  device:updateSoundNodeDebug()
end

local function updateSoundNodeDebug(device)
  bdebug.clearTypeNodeDebugText("CombustionEngine " .. device.name)
  for _, soundData in pairs(device.soundLocations) do
    for _, nodeCid in ipairs(soundData.nodes) do
      bdebug.setNodeDebugText("CombustionEngine " .. device.name, nodeCid, device.name .. ": " .. soundData.text)
    end
  end
end

local function getSoundConfiguration(device)
  return device.soundConfiguration
end

local function setExhaustGainMufflingOffset(device, mufflingOffset, gainOffset)
  if not (device.soundConfiguration and device.soundConfiguration.exhaust) then
    return
  end

  local currentConfig = device.soundConfiguration.exhaust
  currentConfig.params.mufflingOffset = mufflingOffset
  currentConfig.params.gainOffset = gainOffset

  device:setEngineSoundParameterList(device.engineSoundIDExhaust, currentConfig.params, "exhaust")
end

local function setExhaustGainMufflingOffsetRevLimiter(device, mufflingOffset, gainOffset)
  if not (device.soundConfiguration and device.soundConfiguration.exhaust) then
    return
  end

  local currentConfig = device.soundConfiguration.exhaust
  currentConfig.params.mufflingOffsetRevLimiter = mufflingOffset
  currentConfig.params.gainOffsetRevLimiter = gainOffset

  device:setEngineSoundParameterList(device.engineSoundIDExhaust, currentConfig.params, "exhaust")
end

local function resetSounds(device, jbeamData)
  if not sounds.usesOldCustomSounds then
    if jbeamData.soundConfig then
      local soundConfig = v.data[jbeamData.soundConfig]
      if soundConfig then
        device.soundRPMSmoother:reset()
        device.soundLoadSmoother:reset()
        device.engineVolumeCoef = 1
        --dump(sounds)
        sounds.disableOldEngineSounds()
      else
        log("E", "combustionEngine.init", "Can't find sound config: " .. jbeamData.soundConfig)
      end
      if device.engineSoundIDExhaust then
        local endNodeIDPairs
        local maxExhaustAudioOpennessCoef = 0
        local maxExhaustAudioGain
        if device.thermals.exhaustEndNodes and #device.thermals.exhaustEndNodes > 0 then
          endNodeIDPairs = {}
          for _, v in pairs(device.thermals.exhaustEndNodes) do
            maxExhaustAudioOpennessCoef = min(max(maxExhaustAudioOpennessCoef, v.exhaustAudioOpennessCoef), 1)
            maxExhaustAudioGain = maxExhaustAudioGain and max(maxExhaustAudioGain, v.exhaustAudioGainChange) or v.exhaustAudioGainChange
            table.insert(endNodeIDPairs, {v.start, v.finish})
          end
        else
          endNodeIDPairs = {{device.engineNodeID, device.engineNodeID}}
          maxExhaustAudioGain = 0
        end
        device:setExhaustSoundNodes(device.engineSoundIDExhaust, endNodeIDPairs)
        local params = {
          base_muffled = device.exhaustAudioMufflingMinCoef + device.exhaustAudioMufflingCoefRange * (1 - maxExhaustAudioOpennessCoef),
          base_gain = device.exhaustMainGain + maxExhaustAudioGain,
          gainOffset = 0,
          mufflingOffset = 0,
          mufflingOffsetRevLimiter = 0,
          gainOffsetRevLimiter = 0,
          triggerAntilag = 0
        }
        device:setEngineSoundParameterList(device.engineSoundIDExhaust, params, "exhaust")
      end
    end
  else
    log("W", "combustionEngine.init", "Disabling new sounds, found old custom engine sounds...")
  end

  device.turbocharger.resetSounds(v.data[jbeamData.turbocharger])
  device.supercharger.resetSounds(v.data[jbeamData.supercharger])
  device.nitrousOxideInjection.resetSounds(v.data[jbeamData.nitrousOxideInjection])
  device.thermals.resetSounds(jbeamData)
end

local function reset(device, jbeamData)
  local spawnWithEngineRunning = device.spawnVehicleIgnitionLevel > 2
  local spawnWithIgnitionOn = device.spawnVehicleIgnitionLevel > 1

  --reset output AVs and torques
  for i = 1, device.activeOutputPortCount do
    local outputPort = device.activeOutputPorts[i]
    device[device.outputTorqueNames[outputPort]] = 0
    device[device.outputAVNames[outputPort]] = spawnWithEngineRunning and (jbeamData.idleRPM * rpmToAV) or 0
  end
  device.outputRPM = device.outputAV1 * avToRPM
  device.lastOutputAV1 = device.outputAV1
  device.ignitionCoef = spawnWithIgnitionOn and 1 or 0

  device.friction = jbeamData.friction or 0
  device.inputAV = 0
  device.virtualMassAV = 0
  device.isBroken = false
  device.combustionTorque = 0
  device.frictionTorque = 0
  device.nitrousOxideTorque = 0

  device.electricsThrottleName = jbeamData.electricsThrottleName or "throttle"
  device.electricsThrottleFactorName = jbeamData.electricsThrottleFactorName or "throttleFactor"
  device.throttleFactor = 1

  device.throttle = 0
  device.requestedThrottle = 0
  device.dynamicFriction = jbeamData.dynamicFriction or 0
  device.maxTorqueLimit = math.huge

  device.idleAVOverwrite = 0
  device.idleAVReadError = 0
  device.idleAVStartOffset = 0
  device.idleThrottle = 0
  device.idleThrottleTarget = 0
  device.inertia = device.initialInertia
  device.invEngInertia = 1 / device.inertia
  device.halfInvEngInertia = device.invEngInertia * 0.5

  device.starterIgnitionErrorSmoother:reset()
  device.starterIgnitionErrorTimer = 0
  device.starterIgnitionErrorChance = 0.0
  device.starterIgnitionErrorCoef = 1

  device.slowIgnitionErrorSmoother:reset()
  device.slowIgnitionErrorTimer = 0
  device.slowIgnitionErrorChance = 0.0
  device.slowIgnitionErrorCoef = 1
  device.fastIgnitionErrorSmoother:reset()
  device.fastIgnitionErrorChance = 0.0
  device.fastIgnitionErrorCoef = 1

  device.starterEngagedCoef = 0
  device.starterThrottleKillCoef = 1
  device.starterThrottleKillCoefSmoother:set(0)
  device.starterThrottleKillTimer = 0
  device.starterThrottleKillTimerStart = 0
  device.starterDisabled = false
  device.idleAVStartOffsetSmoother:reset()
  device.shutOffSoundRequested = false

  device.stallTimer = 1
  device.isStalled = false

  -- Initialize flood-related variables
  device.floodTimer = 0
  device.floodLevel = 0
  device.isFlooded = false
  device.roughRunning = false
  device.roughRunningIntensity = 0
  device.prevFloodPercent = 0

  -- Reset carburetor
  device.carburetor = carburetor.new(device) or nil
  if device.carburetor then
    device.carburetor:initialize()
  end

  device.forcedInductionCoef = 1
  device.intakeAirDensityCoef = 1
  device.outputTorqueState = 1
  device.outputAVState = 1
  device.isDisabled = false
  device.lastOutputTorque = 0

  -- Reset stall buzzer
  if device.stallBuzzerSoundID then
    obj:stopSFX(device.stallBuzzerSoundID)
  end
  device.stallBuzzerActive = false

  device.loadSmoother:reset()
  device.throttleSmoother:reset()
  device.engineLoad = 0
  device.instantEngineLoad = 0
  device.exhaustFlowCoef = 0
  device.ignitionCutTime = 0
  device.slowIgnitionErrorCoef = 1
  device.fastIgnitionErrorCoef = 1
  device.compressionBrakeCoefDesired = 0
  device.compressionBrakeCoefActual = 0
  device.antiLagCoefDesired = 0
  device.antiLagCoefActual = 0

  device.sustainedAfterFireTimer = 0
  device.instantAfterFireFuel = 0
  device.sustainedAfterFireFuel = 0
  device.shiftAfterFireFuel = 0
  device.continuousAfterFireFuel = 0
  device.instantAfterFireFuelDelay:reset()
  device.sustainedAfterFireFuelDelay:reset()

  -- Reset vapor lock variables
  device.vaporLockTimer = 0
  device.vaporLockLevel = 0
  device.isVaporLocked = false

  device.overRevDamage = 0
  device.overTorqueDamage = 0

  device.engineWorkPerUpdate = 0
  device.frictionLossPerUpdate = 0
  device.pumpingLossPerUpdate = 0
  device.spentEnergy = 0
  device.spentEnergyNitrousOxide = 0
  device.storageWithEnergyCounter = 0
  device.registeredEnergyStorages = {}
  device.previousEnergyLevels = {}
  device.energyStorageRatios = {}
  device.hasFuel = true
  device.remainingFuelRatio = 1

  device.revLimiterActive = false
  device.revLimiterWasActiveTimer = 999

  device.brakeSpecificFuelConsumption = 0

  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1
  device.wearDynamicFrictionCoef = 1
  device.damageDynamicFrictionCoef = 1
  device.wearIdleAVReadErrorRangeCoef = 1
  device.damageIdleAVReadErrorRangeCoef = 1

  device:resetTempRevLimiter()

  device.thermals.reset(jbeamData)

  device.turbocharger.reset(v.data[jbeamData.turbocharger])
  device.supercharger.reset(v.data[jbeamData.supercharger])
  device.nitrousOxideInjection.reset(jbeamData)

  device.torqueData = getTorqueData(device)
  device.maxPower = device.torqueData.maxPower
  device.maxTorque = device.torqueData.maxTorque
  device.maxPowerThrottleMap = device.torqueData.maxPower * psToWatt

  damageTracker.setDamage("engine", "engineDisabled", false)
  damageTracker.setDamage("engine", "engineLockedUp", false)
  damageTracker.setDamage("engine", "engineReducedTorque", false)
  damageTracker.setDamage("engine", "catastrophicOverrevDamage", false)
  damageTracker.setDamage("engine", "mildOverrevDamage", false)
  damageTracker.setDamage("engine", "overRevDanger", false)
  damageTracker.setDamage("engine", "catastrophicOverTorqueDamage", false)
  damageTracker.setDamage("engine", "overTorqueDanger", false)
  damageTracker.setDamage("engine", "engineHydrolocked", false)
  damageTracker.setDamage("engine", "engineIsHydrolocking", false)
  damageTracker.setDamage("engine", "impactDamage", false)

  selectUpdates(device)
end

local function initSounds(device, jbeamData)
  local exhaustEndNodes = device.thermals.exhaustEndNodes or {}

  device.engineMiscSounds = {
    starterSoundEngine = obj:createSFXSource2(jbeamData.starterSample or "event:>Engine>Starter>Old_V2", "AudioDefaultLoop3D", "", device.engineNodeID, 0),
    starterVolume = jbeamData.starterVolume or 1,
    starterVolumeExhaust = jbeamData.starterVolumeExhaust or 1,
    shutOffVolumeEngine = jbeamData.shutOffVolumeEngine or 1,
    shutOffVolumeExhaust = jbeamData.shutOffVolumeExhaust or 1
  }
  
  -- <<< Initialize Stall Buzzer Sound >>>
  if device.stallBuzzerSample and device.stallBuzzerSample ~= "" then
    -- Use engineNodeID as the source location
    device.stallBuzzerSoundID = obj:createSFXSource2(device.stallBuzzerSample, "AudioDefaultLoop3D", "", device.engineNodeID, 0)
    if device.stallBuzzerSoundID then
        obj:setVolume(device.stallBuzzerSoundID, device.stallBuzzerVolume) -- Set volume ONCE here
        log('INFO', 'combustionEngine.initSounds', 'Initialized stall buzzer sound: ' .. device.stallBuzzerSample)
    else
        log('WARN', 'combustionEngine.initSounds', 'Failed to create stall buzzer sound source for: ' .. device.stallBuzzerSample)
    end
  end
  -- <<< END >>>  

  if jbeamData.starterSampleExhaust then
    local starterExhaustNode = #exhaustEndNodes > 0 and exhaustEndNodes[1].finish or device.engineNodeID
    device.engineMiscSounds.starterSoundExhaust = obj:createSFXSource2(jbeamData.starterSampleExhaust, "AudioDefaultLoop3D", "", starterExhaustNode, 0)
    obj:setVolume(device.engineMiscSounds.starterSoundExhaust, device.engineMiscSounds.starterVolumeExhaust)
  end

  if jbeamData.shutOffSampleEngine then
    local shutOffEngineNode = device.engineNodeID or 0
    device.engineMiscSounds.shutOffSoundEngine = obj:createSFXSource2(jbeamData.shutOffSampleEngine, "AudioDefaultLoop3D", "", shutOffEngineNode, 0)
    obj:setVolume(device.engineMiscSounds.shutOffSoundEngine, device.engineMiscSounds.shutOffVolumeEngine)
  end

  if jbeamData.shutOffSampleExhaust then
    local shutOffExhaustNode = #exhaustEndNodes > 0 and exhaustEndNodes[1].finish or device.engineNodeID
    device.engineMiscSounds.shutOffSoundExhaust = obj:createSFXSource2(jbeamData.shutOffSampleExhaust, "AudioDefaultLoop3D", "", shutOffExhaustNode, 0)
    obj:setVolume(device.engineMiscSounds.shutOffSoundExhaust, device.engineMiscSounds.shutOffVolumeExhaust)
  end

  if not sounds.usesOldCustomSounds then
    local hasNewSounds = false
    if jbeamData.soundConfig then
      device.soundConfiguration = {}
      local soundConfig = v.data[jbeamData.soundConfig]

      if soundConfig then
        device.engineSoundID = powertrain.getEngineSoundID()
        device.soundMaxLoadMix = soundConfig.maxLoadMix or 1
        device.soundMinLoadMix = soundConfig.minLoadMix or 0
        local onLoadGain = soundConfig.onLoadGain or 1
        local offLoadGain = soundConfig.offLoadGain or 1
        local fundamentalFrequencyCylinderCount = soundConfig.fundamentalFrequencyCylinderCount or 6
        device.engineVolumeCoef = 1

        local sampleName = soundConfig.sampleName
        local sampleFolder = soundConfig.sampleFolder or "art/sound/blends/"
        local samplePath = sampleFolder .. sampleName .. ".sfxBlend2D.json"

        local engineNodeIDs = {device.engineNodeID} --Hardcode intake sound location to a single node, no need for multiple
        device:initEngineSound(device.engineSoundID, samplePath, engineNodeIDs, offLoadGain, onLoadGain, "engine")

        local main_gain = soundConfig.mainGain or 0

        local eq_a_freq = sounds.hzToFMODHz(soundConfig.lowShelfFreq or soundConfig.lowCutFreq or 20)
        local eq_a_gain = soundConfig.lowShelfGain or 0
        local eq_b_freq = sounds.hzToFMODHz(soundConfig.highShelfFreq or soundConfig.highCutFreq or 10000)
        local eq_b_gain = soundConfig.highShelfGain or 0
        local eq_c_freq = sounds.hzToFMODHz(soundConfig.eqLowFreq or 500)
        local eq_c_gain = soundConfig.eqLowGain or 0
        local eq_c_reso = soundConfig.eqLowWidth or 0
        local eq_d_freq = sounds.hzToFMODHz(soundConfig.eqHighFreq or 2000)
        local eq_d_gain = soundConfig.eqHighGain or 0
        local eq_d_reso = soundConfig.eqHighWidth or 0
        local eq_e_gain = soundConfig.eqFundamentalGain or 0

        local enginePlacement = jbeamData.enginePlacement or "outside"
        local c_enginePlacement = 0
        if enginePlacement == "outside" then
          c_enginePlacement = 0
        elseif enginePlacement == "inside" then
          c_enginePlacement = 1
        end

        local intakeMuffling = soundConfig.intakeMuffling or 1

        -- Audio Debug (engine)
        -- print (string.format("       ENGINE idleRPM = %4.0f / maxRPM = %5.0f", jbeamData.idleRPM, jbeamData.maxRPM))
        -- print (string.format("       ENGINE idleRPM = %4.0f / limiterRPM = %5.0f / maxRPM = %5.0f", jbeamData.idleRPM, jbeamData.revLimiterRPM, jbeamData.maxRPM))
        -- print (string.format("%s  / maingain %4.2fdB / Muffling %.2f / onLoadGain %.2f / offLoadGain %.2f / lowShelf %.0f %4.2fdB / highShelf %4.0f %.2fdB / eqLow %.0f %.2fdB/ eqHigh %4.0f %.2fdB / eqFundamental %.2fdB", sampleName, main_gain, intakeMuffling, onLoadGain, offLoadGain, eq_a_freq, eq_a_gain, eq_b_freq, eq_b_gain, eq_c_freq, eq_c_gain, eq_d_freq, eq_d_gain, eq_e_gain))

        local params = {
          base_gain = main_gain,
          main_gain = 0,
          eq_a_freq = eq_a_freq,
          eq_a_gain = eq_a_gain,
          eq_b_freq = eq_b_freq,
          eq_b_gain = eq_b_gain,
          eq_c_freq = eq_c_freq,
          eq_c_gain = eq_c_gain,
          eq_c_reso = eq_c_reso,
          eq_d_freq = eq_d_freq,
          eq_d_gain = eq_d_gain,
          eq_d_reso = eq_d_reso,
          eq_e_gain = eq_e_gain,
          onLoadGain = onLoadGain,
          offLoadGain = offLoadGain,
          base_muffled = intakeMuffling,
          muffled = 0,
          gainOffset = 0,
          mufflingOffset = 0,
          mufflingOffsetRevLimiter = 0,
          gainOffsetRevLimiter = 0,
          fundamentalFrequencyRPMCoef = fundamentalFrequencyCylinderCount / 120,
          c_enginePlacement = c_enginePlacement,
          compression_brake_coef = 0
        }
        --dump(params)
        device:setEngineSoundParameterList(device.engineSoundID, params, "engine")
        --dump(sounds)
        hasNewSounds = true
      else
        log("E", "combustionEngine.init", "Can't find sound config: " .. jbeamData.soundConfig)
      end
    end
    if jbeamData.soundConfigExhaust then
      device.soundConfiguration = device.soundConfiguration or {}
      local soundConfig = v.data[jbeamData.soundConfigExhaust]
      if soundConfig then
        device.engineSoundIDExhaust = powertrain.getEngineSoundID()
        device.soundMaxLoadMixExhaust = soundConfig.maxLoadMix
        device.soundMinLoadMixExhaust = soundConfig.minLoadMix
        local onLoadGain = soundConfig.onLoadGain or 1
        local offLoadGain = soundConfig.offLoadGain or 1
        local fundamentalFrequencyCylinderCount = soundConfig.fundamentalFrequencyCylinderCount or 6
        device.engineVolumeCoef = 1

        local sampleName = soundConfig.sampleName
        local sampleFolder = soundConfig.sampleFolder or "art/sound/blends/"
        local samplePath = sampleFolder .. sampleName .. ".sfxBlend2D.json"

        local endNodeIDPairs

        device.exhaustAudioMufflingMinCoef = soundConfig.exhaustAudioMufflingBaseCoef or 0
        device.exhaustAudioMufflingCoefRange = 1 - device.exhaustAudioMufflingMinCoef
        local maxExhaustAudioOpennessCoef = 0
        local maxExhaustAudioGain
        if #exhaustEndNodes > 0 then
          endNodeIDPairs = {}
          for _, v in pairs(exhaustEndNodes) do
            maxExhaustAudioOpennessCoef = min(max(maxExhaustAudioOpennessCoef, v.exhaustAudioOpennessCoef), 1)
            maxExhaustAudioGain = maxExhaustAudioGain and max(maxExhaustAudioGain, v.exhaustAudioGainChange) or v.exhaustAudioGainChange --we want the biggest number, ie the least amount of muffling
            table.insert(endNodeIDPairs, {v.start, v.finish})
          end
        else
          endNodeIDPairs = {{device.engineNodeID, device.engineNodeID}}
          maxExhaustAudioGain = 0
        end
        device:initExhaustSound(device.engineSoundIDExhaust, samplePath, endNodeIDPairs, offLoadGain, onLoadGain, "exhaust")

        device.exhaustMainGain = soundConfig.mainGain or 0
        local main_gain = device.exhaustMainGain + maxExhaustAudioGain

        local eq_a_freq = sounds.hzToFMODHz(soundConfig.lowShelfFreq or soundConfig.lowCutFreq or 20)
        local eq_a_gain = soundConfig.lowShelfGain or 0
        local eq_b_freq = sounds.hzToFMODHz(soundConfig.highShelfFreq or soundConfig.highCutFreq or 10000)
        local eq_b_gain = soundConfig.highShelfGain or 0
        local eq_c_freq = sounds.hzToFMODHz(soundConfig.eqLowFreq or 500)
        local eq_c_gain = soundConfig.eqLowGain or 0
        local eq_c_reso = soundConfig.eqLowWidth or 0
        local eq_d_freq = sounds.hzToFMODHz(soundConfig.eqHighFreq or 2000)
        local eq_d_gain = soundConfig.eqHighGain or 0
        local eq_d_reso = soundConfig.eqHighWidth or 0
        local eq_e_gain = soundConfig.eqFundamentalGain or 0

        local exhaustMuffling = device.exhaustAudioMufflingMinCoef + device.exhaustAudioMufflingCoefRange * (1 - maxExhaustAudioOpennessCoef)

        -- Audio Debug (exhaust)
        -- print (string.format("%s / maingain %4.2fdB / Muffling %.2f / onLoadGain %.2f / offLoadGain %.2f / lowShelf %.0fhz %4.2fdB / highShelf %4.0fhz %.2fdB / eqLow %.0fhz %.2fdB/ eqHigh %4.0fhz %.2fdB / eqFundamental %.2fdB ",sampleName, main_gain, exhaustMuffling, onLoadGain, offLoadGain, eq_a_freq, eq_a_gain, eq_b_freq, eq_b_gain, eq_c_freq, eq_c_gain, eq_d_freq, eq_d_gain, eq_e_gain))

        local params = {
          base_gain = main_gain,
          main_gain = 0,
          eq_a_freq = eq_a_freq,
          eq_a_gain = eq_a_gain,
          eq_b_freq = eq_b_freq,
          eq_b_gain = eq_b_gain,
          eq_c_freq = eq_c_freq,
          eq_c_gain = eq_c_gain,
          eq_c_reso = eq_c_reso,
          eq_d_freq = eq_d_freq,
          eq_d_gain = eq_d_gain,
          eq_d_reso = eq_d_reso,
          eq_e_gain = eq_e_gain,
          onLoadGain = onLoadGain,
          offLoadGain = offLoadGain,
          base_muffled = exhaustMuffling,
          muffled = 0,
          gainOffset = 0,
          mufflingOffset = 0,
          mufflingOffsetRevLimiter = 0,
          gainOffsetRevLimiter = 0,
          triggerAntilag = 0,
          fundamentalFrequencyRPMCoef = fundamentalFrequencyCylinderCount / 120
        }
        --dump(params)

        device:setEngineSoundParameterList(device.engineSoundIDExhaust, params, "exhaust")
        hasNewSounds = true
      else
        log("E", "combustionEngine.init", "Can't find sound config: " .. jbeamData.soundConfigExhaust)
      end
    end

    if hasNewSounds then
      local rpmInRate = jbeamData.rpmSmootherInRate or 15
      local rpmOutRate = jbeamData.rpmSmootherOutRate or 25
      device.soundRPMSmoother = newTemporalSmoothingNonLinear(rpmInRate, rpmOutRate)
      local loadInRate = jbeamData.loadSmootherInRate or 20
      local loadOutRate = jbeamData.loadSmootherOutRate or 20
      device.soundLoadSmoother = newTemporalSmoothingNonLinear(loadInRate, loadOutRate)

      device.updateSounds = updateSounds
      sounds.disableOldEngineSounds()
    end
  else
    log("W", "combustionEngine.initSounds", "Disabling new sounds, found old custom engine sounds...")
  end

  device.turbocharger.initSounds(v.data[jbeamData.turbocharger])
  device.supercharger.initSounds(v.data[jbeamData.supercharger])
  device.nitrousOxideInjection.initSounds(v.data[jbeamData.nitrousOxideInjection])
  device.thermals.initSounds(jbeamData)
end

local function new(jbeamData)
  local device = {
    deviceCategories = shallowcopy(M.deviceCategories),
    requiredExternalInertiaOutputs = shallowcopy(M.requiredExternalInertiaOutputs),
    outputPorts = shallowcopy(M.outputPorts),
    name = jbeamData.name,
    type = jbeamData.type,
    inputName = jbeamData.inputName,
    inputIndex = jbeamData.inputIndex,
    friction = jbeamData.friction or 0,
    cumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    isPropulsed = true,
    inputAV = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    isBroken = false,
    combustionTorque = 0,
    frictionTorque = 0,
    nitrousOxideTorque = 0,
    electricsThrottleName = jbeamData.electricsThrottleName or "throttle",
    electricsThrottleFactorName = jbeamData.electricsThrottleFactorName or "throttleFactor",
    throttleFactor = 1,
    throttle = 0,
    requestedThrottle = 0,
    maxTorqueLimit = math.huge,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    idleRPM = jbeamData.idleRPM,
    idleAV = jbeamData.idleRPM * rpmToAV,
    idleAVOverwrite = 0,
    idleAVStartOffset = 0,
    idleAVReadError = 0,
    idleAVReadErrorRange = (jbeamData.idleRPMRoughness or 50) * rpmToAV,
    idleThrottle = 0,
    idleThrottleTarget = 0,
    maxIdleThrottle = clamp(jbeamData.maxIdleThrottle or 0.15, 0, 1),
    maxIdleThrottleOverwrite = 0,
    idleTime = 1 / (max(jbeamData.idleUpdateFrequency or 100, 0.1)),
    idleTimeRandomness = clamp(jbeamData.idleUpdateFrequencyRandomness or 0.01, 0, 1),
    idleTimer = 0,
    idleControllerP = jbeamData.idleControllerP or 0.01,
    idleThrottleSmoother = newTemporalSmoothing(jbeamData.idleSmoothingDown or 100, jbeamData.idleSmoothingUp or 100),
    maxRPM = jbeamData.maxRPM,
    maxAV = jbeamData.maxRPM * rpmToAV,
    inertia = jbeamData.inertia or 0.1,
    starterTorque = jbeamData.starterTorque or (jbeamData.friction * 5),
    starterMaxAV = jbeamData.starterMaxRPM or  (jbeamData.idleRPM * 0.6) * rpmToAV,
    shutOffSoundRequested = false,
    starterEngagedCoef = 0,
    starterThrottleKillCoef = 1,
    starterThrottleKillCoefSmoother = newTemporalSmoothing(70, 40),
    starterThrottleKillTimer = 0,
    starterThrottleKillTimerStart = 0,
    starterThrottleKillTime = jbeamData.starterThrottleKillTime or 0.5,
    starterDisabled = false,
    stallTimer = 1,
    isStalled = false,
    -- Initialize flood-related variables
    floodTimer = 0,
    floodLevel = 0,
    isFlooded = false,
    roughRunning = false,
    roughRunningIntensity = 0,
    prevFloodPercent = 0,
    -- Initialize vapor lock variables
    vaporLockTimer = 0,
    vaporLockLevel = 0,
    isVaporLocked = false,
    particulates = jbeamData.particulates,
    thermalsEnabled = jbeamData.thermalsEnabled,
    engineBlockMaterial = jbeamData.engineBlockMaterial,
    oilVolume = jbeamData.oilVolume,
    cylinderWallTemperatureDamageThreshold = jbeamData.cylinderWallTemperatureDamageThreshold,
    headGasketDamageThreshold = jbeamData.headGasketDamageThreshold,
    pistonRingDamageThreshold = jbeamData.pistonRingDamageThreshold,
    connectingRodDamageThreshold = jbeamData.connectingRodDamageThreshold,
    forcedInductionCoef = 1,
    intakeAirDensityCoef = 1,
    outputTorqueState = 1,
    outputAVState = 1,
    isDisabled = false,
    lastOutputTorque = 0,
    loadSmoother = newTemporalSmoothing(2, 2),
    throttleSmoother = newTemporalSmoothing(30, 15),
    engineLoad = 0,
    instantEngineLoad = 0,
    exhaustFlowCoef = 0,
    revLimiterActiveMaxExhaustFlowCoef = jbeamData.revLimiterActiveMaxExhaustFlowCoef or 0.5,
    ignitionCutTime = 0,
    slowIgnitionErrorCoef = 1,
    fastIgnitionErrorCoef = 1,
    instantAfterFireCoef = jbeamData.instantAfterFireCoef or 0,
    sustainedAfterFireCoef = jbeamData.sustainedAfterFireCoef or 0,
    sustainedAfterFireTimer = 0,
    sustainedAfterFireTime = jbeamData.sustainedAfterFireTime or 1.5,
    instantAfterFireFuel = 0,
    sustainedAfterFireFuel = 0,
    shiftAfterFireFuel = 0,
    continuousAfterFireFuel = 0,
    instantAfterFireFuelDelay = delayLine.new(0.1),
    sustainedAfterFireFuelDelay = delayLine.new(0.3),
    exhaustFlowDelay = delayLine.new(0.1),
    antiLagCoefDesired = 0,
    antiLagCoefActual = 0,
    overRevDamage = 0,
    maxOverRevDamage = jbeamData.maxOverRevDamage or 1500,
    maxTorqueRating = jbeamData.maxTorqueRating or -1,
    overTorqueDamage = 0,
    maxOverTorqueDamage = jbeamData.maxOverTorqueDamage or 1000,
    engineWorkPerUpdate = 0,
    frictionLossPerUpdate = 0,
    pumpingLossPerUpdate = 0,
    spentEnergy = 0,
    spentEnergyNitrousOxide = 0,
    storageWithEnergyCounter = 0,
    registeredEnergyStorages = {},
    previousEnergyLevels = {},
    energyStorageRatios = {},
    hasFuel = true,
    remainingFuelRatio = 1,
    fixedStepTimer = 0,
    fixedStepTime = 1 / 100,
    soundLocations = {},
    stallBuzzerSample = jbeamData.stallBuzzerSample or "mods/unpacked/BetterStarter/art/sound/buzzer.wav", -- Default path adjusted
    stallBuzzerVolume = jbeamData.stallBuzzerVolume or 0.4,
    stallBuzzerCrankingPitch = jbeamData.stallBuzzerCrankingPitch or 0.1,
    stallBuzzerSoundID = nil,
    --
    --wear/damage modifiers
    wearFrictionCoef = 1,
    damageFrictionCoef = 1,
    wearDynamicFrictionCoef = 1,
    damageDynamicFrictionCoef = 1,
    wearIdleAVReadErrorRangeCoef = 1,
    damageIdleAVReadErrorRangeCoef = 1,
    --
    --methods
    initSounds = initSounds,
    resetSounds = resetSounds,
    setExhaustGainMufflingOffset = setExhaustGainMufflingOffset,
    setExhaustGainMufflingOffsetRevLimiter = setExhaustGainMufflingOffsetRevLimiter,
    updateFloodingState = updateFloodingState,
    reset = reset,
    onBreak = onBreak,
    beamBroke = beamBroke,
    validate = validate,
    calculateInertia = calculateInertia,
    updateGFX = updateGFX,
    updateFixedStep = updateFixedStep,
    updateSounds = nil,
    scaleFriction = scaleFriction,
    scaleFrictionInitial = scaleFrictionInitial,
    scaleOutputTorque = scaleOutputTorque,
    activateStarter = activateStarter,
    deactivateStarter = deactivateStarter,
    setCompressionBrakeCoef = setCompressionBrakeCoef,
    setAntilagCoef = setAntilagCoef,
    sendTorqueData = sendTorqueData,
    getTorqueData = getTorqueData,
    checkHydroLocking = checkHydroLocking,
    lockUp = lockUp,
    disable = disable,
    enable = enable,
    setIgnition = setIgnition,
    cutIgnition = cutIgnition,
    setTempRevLimiter = setTempRevLimiter,
    resetTempRevLimiter = resetTempRevLimiter,
    updateFuelUsage = updateFuelUsage,
    updateEnergyStorageRatios = updateEnergyStorageRatios,
    registerStorage = registerStorage,
    setExhaustSoundNodes = setExhaustSoundNodes,
    exhaustEndNodesChanged = exhaustEndNodesChanged,
    initEngineSound = initEngineSound,
    initExhaustSound = initExhaustSound,
    setEngineSoundParameter = setEngineSoundParameter,
    setEngineSoundParameterList = setEngineSoundParameterList,
    getSoundConfiguration = getSoundConfiguration,
    setSoundLocation = setSoundLocation,
    updateSoundNodeDebug = updateSoundNodeDebug,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  device.spawnVehicleIgnitionLevel = electrics.values.ignitionLevel
  local spawnWithIgnitionOn = device.spawnVehicleIgnitionLevel > 1
  
  --this code handles the requirement to support multiple output clutches
  --by default the engine has only one output, we need to know the number before building the tree, so it needs to be specified in jbeam
  device.numberOfOutputPorts = jbeamData.numberOfOutputPorts or 1
  device.outputPorts = {} --reset the defined outputports
  device.outputTorqueNames = {}
  device.outputAVNames = {}
  for i = 1, device.numberOfOutputPorts, 1 do
    device.outputPorts[i] = true --let powertrain know which outputports we support
  end

  device.ignitionCoef = spawnWithIgnitionOn and 1 or 0
  device.lastIgnitionCoef = device.ignitionCoef  -- Initialize to same value as ignitionCoef
  device.invStarterMaxAV = 1 / device.starterMaxAV

  device.initialFriction = device.friction
  device.engineBrakeTorque = jbeamData.engineBrakeTorque or device.friction * 2

  local torqueReactionNodes_nodes = jbeamData.torqueReactionNodes_nodes
  if torqueReactionNodes_nodes and type(torqueReactionNodes_nodes) == "table" then
    local hasValidReactioNodes = true
    for _, v in pairs(torqueReactionNodes_nodes) do
      if type(v) ~= "number" then
        hasValidReactioNodes = false
      end
    end
    if hasValidReactioNodes then
      device.torqueReactionNodes = torqueReactionNodes_nodes
    end
  end
  if not device.torqueReactionNodes then
    device.torqueReactionNodes = {-1, -1, -1}
  end

  device.waterDamageNodes = jbeamData.waterDamage and jbeamData.waterDamage._engineGroup_nodes or {}

  device.canFlood = device.waterDamageNodes and type(device.waterDamageNodes) == "table" and #device.waterDamageNodes > 0

  device.maxPhysicalAV = (jbeamData.maxPhysicalRPM or (jbeamData.maxRPM * 1.05)) * rpmToAV --what the engine is physically capable of

  if not jbeamData.torque then
    log("E", "combustionEngine.init", "Can't find torque table... Powertrain is going to break!")
  end

  local baseTorqueTable = tableFromHeaderTable(jbeamData.torque)
  local rawBasePoints = {}
  local maxAvailableRPM = 0
  for _, v in pairs(baseTorqueTable) do
    maxAvailableRPM = max(maxAvailableRPM, v.rpm)
    table.insert(rawBasePoints, {v.rpm, v.torque})
    -- print (string.format("RPM = %5.0f, TORQUE = %4.0f", v.rpm, v.torque))
  end
  local rawBaseCurve = createCurve(rawBasePoints)

  local rawTorqueMultCurve = {}
  if jbeamData.torqueModMult then
    local multTorqueTable = tableFromHeaderTable(jbeamData.torqueModMult)
    local rawTorqueMultPoints = {}
    for _, v in pairs(multTorqueTable) do
      maxAvailableRPM = max(maxAvailableRPM, v.rpm)
      table.insert(rawTorqueMultPoints, {v.rpm, v.torque})
    end
    rawTorqueMultCurve = createCurve(rawTorqueMultPoints)
  end

  local rawIntakeCurve = {}
  local lastRawIntakeValue = 0
  if jbeamData.torqueModIntake then
    local intakeTorqueTable = tableFromHeaderTable(jbeamData.torqueModIntake)
    local rawIntakePoints = {}
    for _, v in pairs(intakeTorqueTable) do
      maxAvailableRPM = max(maxAvailableRPM, v.rpm)
      table.insert(rawIntakePoints, {v.rpm, v.torque})
    end
    rawIntakeCurve = createCurve(rawIntakePoints)
    lastRawIntakeValue = rawIntakeCurve[#rawIntakeCurve]
  end

  local rawExhaustCurve = {}
  local lastRawExhaustValue = 0
  if jbeamData.torqueModExhaust then
    local exhaustTorqueTable = tableFromHeaderTable(jbeamData.torqueModExhaust)
    local rawExhaustPoints = {}
    for _, v in pairs(exhaustTorqueTable) do
      maxAvailableRPM = max(maxAvailableRPM, v.rpm)
      table.insert(rawExhaustPoints, {v.rpm, v.torque})
    end
    rawExhaustCurve = createCurve(rawExhaustPoints)
    lastRawExhaustValue = rawExhaustCurve[#rawExhaustCurve]
  end

  local rawCombinedCurve = {}
  for i = 0, maxAvailableRPM, 1 do
    local base = rawBaseCurve[i] or 0
    local baseMult = rawTorqueMultCurve[i] or 1
    local intake = rawIntakeCurve[i] or lastRawIntakeValue
    local exhaust = rawExhaustCurve[i] or lastRawExhaustValue
    rawCombinedCurve[i] = base * baseMult + intake + exhaust
  end

  device.compressionBrakeCurve = {}
  jbeamData.torqueCompressionBrake = jbeamData.torqueCompressionBrake or {{"rpm", "torque"}, {0, 0}, {1000, 500}, {3000, 1500}} --todo remove defaults
  if jbeamData.torqueCompressionBrake then
    local compressionBrakeTorqueTable = tableFromHeaderTable(jbeamData.torqueCompressionBrake)
    local rawPoints = {}
    for _, v in pairs(compressionBrakeTorqueTable) do
      maxAvailableRPM = max(maxAvailableRPM, v.rpm)
      table.insert(rawPoints, {v.rpm, v.torque})
    end
    device.compressionBrakeCurve = createCurve(rawPoints)
  end
  device.compressionBrakeCoefActual = 0
  device.compressionBrakeCoefDesired = 0

  device.maxAvailableRPM = maxAvailableRPM
  device.maxRPM = min(device.maxRPM, maxAvailableRPM)
  device.maxAV = min(device.maxAV, maxAvailableRPM * rpmToAV)

  device.applyRevLimiter = revLimiterDisabledMethod
  device.revLimiterActive = false
  device.revLimiterWasActiveTimer = 999
  local preRevLimiterMaxRPM = device.maxRPM --we need to save the jbeam defined maxrpm for our torque table/drop off calculations later
  device.hasRevLimiter = jbeamData.hasRevLimiter == nil and true or jbeamData.hasRevLimiter --TBD, default should be "no" rev limiter
  if device.hasRevLimiter then
    device.revLimiterType = jbeamData.revLimiterType or "rpmDrop" --alternatives: "timeBased", "soft"
    --save the revlimiter RPM/AV for use within the limiting functions
    device.revLimiterRPM = jbeamData.revLimiterRPM or device.maxRPM
    device.revLimiterAV = device.revLimiterRPM * rpmToAV
    --make sure that the reported max RPM/AV is the one from the revlimiter, many other subsystems use this value
    device.maxRPM = device.revLimiterRPM
    device.maxAV = device.maxRPM * rpmToAV

    if device.revLimiterType == "rpmDrop" then --purely rpm drop based
      device.revLimiterAVDrop = (jbeamData.revLimiterRPMDrop or (jbeamData.maxRPM * 0.03)) * rpmToAV
      device.applyRevLimiter = revLimiterRPMDropMethod
    elseif device.revLimiterType == "timeBased" then --combined both time or rpm drop, whatever happens first
      device.revLimiterCutTime = jbeamData.revLimiterCutTime or 0.15
      device.revLimiterMaxAVDrop = (jbeamData.revLimiterMaxRPMDrop or 500) * rpmToAV
      device.revLimiterActiveTimer = 0
      device.applyRevLimiter = revLimiterTimeMethod
    elseif device.revLimiterType == "soft" then --soft limiter without any "drop", it just smoothly fades out throttle
      device.revLimiterMaxAVOvershoot = (jbeamData.revLimiterSmoothOvershootRPM or 50) * rpmToAV
      device.revLimiterMaxAV = device.maxAV + device.revLimiterMaxAVOvershoot
      device.invRevLimiterRange = 1 / (device.revLimiterMaxAV - device.maxAV)
      device.applyRevLimiter = revLimiterSoftMethod
    else
      log("E", "combustionEngine.init", "Unknown rev limiter type: " .. device.revLimiterType)
      log("E", "combustionEngine.init", "Rev limiter will be disabled!")
      device.hasRevLimiter = false
    end
  end

  device:resetTempRevLimiter()

  --cut off torque below a certain RPM to help stalling
  for i = 0, device.idleRPM * (device.requiredEnergyType == "gasoline" and 0.5 or 0.3), 1 do
    rawCombinedCurve[i] = 0
  end

  local combinedTorquePoints = {}
  --only use the existing torque table up to our previosuly saved max RPM without rev limiter influence so that the drop off works correctly
  for i = 0, preRevLimiterMaxRPM, 1 do
    table.insert(combinedTorquePoints, {i, rawCombinedCurve[i] or 0})
  end

  --past redline we want to gracefully reduce the torque for a natural redline
  device.redlineTorqueDropOffRange = clamp(jbeamData.redlineTorqueDropOffRange or 500, 10, preRevLimiterMaxRPM)

  --last usable torque value for a smooth transition to past-maxRPM-drop-off
  local rawMaxRPMTorque = rawCombinedCurve[preRevLimiterMaxRPM] or 0

  --create the drop off past the max rpm for a natural redline
  table.insert(combinedTorquePoints, {preRevLimiterMaxRPM + device.redlineTorqueDropOffRange * 0.5, rawMaxRPMTorque * 0.7})
  table.insert(combinedTorquePoints, {preRevLimiterMaxRPM + device.redlineTorqueDropOffRange, rawMaxRPMTorque / 5})
  table.insert(combinedTorquePoints, {preRevLimiterMaxRPM + device.redlineTorqueDropOffRange * 2, 0})

  --if our revlimiter RPM is higher than maxRPM, maxRPM _becomes_ that. This means that we need to make sure the torque table is also filled up to that point
  if preRevLimiterMaxRPM + device.redlineTorqueDropOffRange * 2 < device.maxRPM then
    table.insert(combinedTorquePoints, {device.maxRPM, 0})
  end

  --actually create the final torque curve
  device.torqueCurve = createCurve(combinedTorquePoints)

  device.invEngInertia = 1 / device.inertia
  device.halfInvEngInertia = device.invEngInertia * 0.5

  local idleReadErrorRate = jbeamData.idleRPMRoughnessRate or device.idleAVReadErrorRange * 2
  device.idleAVReadErrorSmoother = newTemporalSmoothing(idleReadErrorRate, idleReadErrorRate)
  device.idleAVReadErrorRangeHalf = device.idleAVReadErrorRange * 0.5
  device.maxIdleAV = device.idleAV + device.idleAVReadErrorRangeHalf
  device.minIdleAV = device.idleAV - device.idleAVReadErrorRangeHalf

  local idleAVStartOffsetRate = jbeamData.idleRPMStartRate or 1
  device.idleAVStartOffsetSmoother = newTemporalSmoothingNonLinear(idleAVStartOffsetRate, 100)
  device.idleStartCoef = jbeamData.idleRPMStartCoef or 2

  device.idleTorque = device.torqueCurve[floor(device.idleRPM)] or 0

  --ignition error properties
  --starter
  device.starterIgnitionErrorSmoother = newTemporalSmoothing(2, 2)
  device.starterIgnitionErrorTimer = 0
  device.starterIgnitionErrorInterval = 5
  device.starterIgnitionErrorChance = 0.0
  device.starterIgnitionErrorCoef = 1
  --slow
  device.slowIgnitionErrorSmoother = newTemporalSmoothing(2, 2)
  device.slowIgnitionErrorTimer = 0
  device.slowIgnitionErrorChance = 0.0
  device.slowIgnitionErrorInterval = 5
  device.slowIgnitionErrorCoef = 1
  --fast
  device.fastIgnitionErrorSmoother = newTemporalSmoothing(10, 10)
  device.fastIgnitionErrorChance = 0.0
  device.fastIgnitionErrorCoef = 1

  device.brakeSpecificFuelConsumption = 0

  local tempBurnEfficiencyTable = nil
  if not jbeamData.burnEfficiency or type(jbeamData.burnEfficiency) == "number" then
    tempBurnEfficiencyTable = {{0, jbeamData.burnEfficiency or 1}, {1, jbeamData.burnEfficiency or 1}}
  elseif type(jbeamData.burnEfficiency) == "table" then
    tempBurnEfficiencyTable = deepcopy(jbeamData.burnEfficiency)
  end

  local copy = deepcopy(tempBurnEfficiencyTable)
  tempBurnEfficiencyTable = {}
  for k, v in pairs(copy) do
    if type(k) == "number" then
      table.insert(tempBurnEfficiencyTable, {v[1] * 100, v[2]})
    end
  end

  tempBurnEfficiencyTable = createCurve(tempBurnEfficiencyTable)
  device.invBurnEfficiencyTable = {}
  device.invBurnEfficiencyCoef = 1
  for k, v in pairs(tempBurnEfficiencyTable) do
    device.invBurnEfficiencyTable[k] = 1 / v
  end

  device.requiredEnergyType = jbeamData.requiredEnergyType or "gasoline"
  device.energyStorage = jbeamData.energyStorage

  if device.torqueReactionNodes and #device.torqueReactionNodes == 3 and device.torqueReactionNodes[1] >= 0 then
    local pos1 = vec3(v.data.nodes[device.torqueReactionNodes[1]].pos)
    local pos2 = vec3(v.data.nodes[device.torqueReactionNodes[2]].pos)
    local pos3 = vec3(v.data.nodes[device.torqueReactionNodes[3]].pos)
    local avgPos = (((pos1 + pos2) / 2) + pos3) / 2
    device.visualPosition = {x = avgPos.x, y = avgPos.y, z = avgPos.z}
  end

  device.engineNodeID = device.torqueReactionNodes and (device.torqueReactionNodes[1] or v.data.refNodes[0].ref) or v.data.refNodes[0].ref
  if device.engineNodeID < 0 then
    log("W", "combustionEngine.init", "Can't find suitable engine node, using ref node instead!")
    device.engineNodeID = v.data.refNodes[0].ref
  end

  device.engineBlockNodes = {}
  if jbeamData.engineBlock and jbeamData.engineBlock._engineGroup_nodes and #jbeamData.engineBlock._engineGroup_nodes >= 2 then
    device.engineBlockNodes = jbeamData.engineBlock._engineGroup_nodes
  end

  --dump(jbeamData)

  local thermalsFileName = jbeamData.thermalsLuaFileName or "powertrain/combustionEngineThermals"
  device.thermals = rerequire(thermalsFileName)
  device.thermals.init(device, jbeamData)

  if jbeamData.turbocharger and v.data[jbeamData.turbocharger] then
    local turbochargerFileName = jbeamData.turbochargerLuaFileName or "powertrain/turbocharger"
    device.turbocharger = rerequire(turbochargerFileName)
    device.turbocharger.init(device, v.data[jbeamData.turbocharger])
  else
    device.turbocharger = {reset = nop, updateGFX = nop, updateFixedStep = nop, updateSounds = nop, initSounds = nop, resetSounds = nop, getPartCondition = nop, isExisting = false}
  end

  if jbeamData.supercharger and v.data[jbeamData.supercharger] then
    local superchargerFileName = jbeamData.superchargerLuaFileName or "powertrain/supercharger"
    device.supercharger = rerequire(superchargerFileName)
    device.supercharger.init(device, v.data[jbeamData.supercharger])
  else
    device.supercharger = {reset = nop, updateGFX = nop, updateFixedStep = nop, updateSounds = nop, initSounds = nop, resetSounds = nop, getPartCondition = nop, isExisting = false}
  end

  if jbeamData.nitrousOxideInjection and v.data[jbeamData.nitrousOxideInjection] then
    local nitrousOxideFileName = jbeamData.nitrousOxideLuaFileName or "powertrain/nitrousOxideInjection"
    device.nitrousOxideInjection = rerequire(nitrousOxideFileName)
    device.nitrousOxideInjection.init(device, v.data[jbeamData.nitrousOxideInjection])
  else
    device.nitrousOxideInjection = {reset = nop, updateGFX = nop, updateSounds = nop, initSounds = nop, resetSounds = nop, registerStorage = nop, getAddedTorque = nop, getPartCondition = nop, isExisting = false}
  end

  device.torqueData = getTorqueData(device)
  device.maxPower = device.torqueData.maxPower
  device.maxTorque = device.torqueData.maxTorque
  device.maxPowerThrottleMap = device.torqueData.maxPower * psToWatt

  device.breakTriggerBeam = jbeamData.breakTriggerBeam
  if device.breakTriggerBeam and device.breakTriggerBeam == "" then
    --get rid of the break beam if it's just an empty string (cancellation)
    device.breakTriggerBeam = nil
  end

  damageTracker.setDamage("engine", "engineDisabled", false)
  damageTracker.setDamage("engine", "engineLockedUp", false)
  damageTracker.setDamage("engine", "engineReducedTorque", false)
  damageTracker.setDamage("engine", "catastrophicOverrevDamage", false)
  damageTracker.setDamage("engine", "mildOverrevDamage", false)
  damageTracker.setDamage("engine", "catastrophicOverTorqueDamage", false)
  damageTracker.setDamage("engine", "mildOverTorqueDamage", false)
  damageTracker.setDamage("engine", "engineHydrolocked", false)
  damageTracker.setDamage("engine", "engineIsHydrolocking", false)
  damageTracker.setDamage("engine", "impactDamage", false)

  selectUpdates(device)

  return device
end

-- Helper function to trigger visual effects for misfires
local function triggerMisfireEffect(device, severity)
  if not device.triggeredEffects then
    device.triggeredEffects = {}
  end
  
  -- Only trigger one effect at a time
  if not device.triggeredEffects.misfire then
    device.triggeredEffects.misfire = true
    
    -- Create a particle effect or other visual feedback
    -- This is a placeholder - replace with actual effect triggering code
    if device.triggerEffect then
      device:triggerEffect("misfire", {
        intensity = severity,
        duration = 0.5,
        position = device.engineNodeID
      })
    end
    
    -- Reset after effect duration
    timer(0.5, function()
      if device.triggeredEffects then
        device.triggeredEffects.misfire = false
      end
    end)
  end
end

-- Helper function to play flooding-related sounds
local function playFloodingSound(device, soundType, intensity)
  if not device.soundEffects then
    device.soundEffects = {}
  end
  
  -- Only play one sound at a time
  if not device.soundEffects.playing then
    device.soundEffects.playing = true
    
    local soundName = ""
    local volume = 0.5 * intensity
    local pitch = 1.0
    
    -- Select appropriate sound based on type and intensity
    if soundType == "flooded" then
      soundName = "event:>Engine>Misfire>Severe"
      volume = 0.6 * intensity
      pitch = 0.9
    elseif soundType == "struggling" then
      soundName = "event:>Engine>Misfire>Medium"
      volume = 0.5 * intensity
      pitch = 1.0
    else -- misfire
      soundName = "event:>Engine>Misfire>Light"
      volume = 0.4 * intensity
      pitch = 1.1
    end
    
    -- Play the sound
    if device.playSound then
      device:playSound({
        name = soundName,
        gain = volume,
        pitch = pitch,
        node = device.engineNodeID
      })
    end
    
    -- Reset sound flag after a delay
    timer(1.5, function()
      if device.soundEffects then
        device.soundEffects.playing = false
      end
    end)
  end
end

-- Update UI with flood status
local function updateFloodStatus(device, status)
  if not device.uiState then
    device.uiState = {}
  end
  
  -- Update local state
  device.uiState.floodStatus = status
  
  -- Notify UI components
  if device.onFloodStatusChanged then
    device:onFloodStatusChanged(status)
  end
  
  -- Log status changes for debugging
  if status.isFlooded and (not device.lastFloodStatus or not device.lastFloodStatus.isFlooded) then
    log('I', 'combustionEngine', 'Engine is now flooded')
  elseif not status.isFlooded and device.lastFloodStatus and device.lastFloodStatus.isFlooded then
    log('I', 'combustionEngine', 'Engine flood condition cleared')
  end
  
  device.lastFloodStatus = deepcopy(status)
end

-- Initialize the flooding system when a new engine is created
local function initFloodingSystem(device)
  -- Assign helper functions
  device.triggerMisfireEffect = function(sev) return triggerMisfireEffect(device, sev) end
  device.playFloodingSound = function(type, int) return playFloodingSound(device, type, int) end
  device.updateFloodStatus = function(status) return updateFloodStatus(device, status) end
  
  -- Initialize timers
  device.lastFloodSoundTime = 0
  
  -- Initialize flood-related variables
  device.floodTimer = 0
  device.floodLevel = 0
  device.isFlooded = false
  device.sparkFouling = 0
  device.starterStress = 0
  
  -- Log initialization
  log('D', 'combustionEngine', 'Flooding system initialized')
  
  return device
end

-- Store original functions
local originalNew = M.new
local originalInit = M.init

-- Create a new constructor that wraps the original one
local function newConstructor(jbeamData)
  -- Call the original constructor if it exists, otherwise create a basic device
  local device = originalNew and originalNew(jbeamData) or {}
  return device
end

-- Override the init function to inject our code
function M.init(device, jbeamData)
  -- Call the original init function first
  if originalInit then
    originalInit(device, jbeamData)
  end
  
  -- Initialize our flood-related fields
  device.floodTimer = 0
  device.floodLevel = 0
  device.isFlooded = false
  device.sparkFouling = 0
  device.starterStress = 0
  device.lastFloodSoundTime = 0
  
  -- Initialize helper functions with safe defaults
  device.triggerMisfireEffect = device.triggerMisfireEffect or function() end
  device.playFloodingSound = device.playFloodingSound or function() end
  device.updateFloodStatus = device.updateFloodStatus or function() end
  
  -- Log successful initialization
  log('D', 'combustionEngine', 'ImprovedStarter: Engine device initialized with flood system')
end

-- Restore the original new function but with our init hook
M.new = new

local command = "obj:queueGameEngineLua(string.format('scenarios.getScenario().wheelDataCallback(%s)', serialize({wheels.wheels[0].absActive, wheels.wheels[0].angularVelocity, wheels.wheels[0].angularVelocityBrakeCouple})))"

return M
