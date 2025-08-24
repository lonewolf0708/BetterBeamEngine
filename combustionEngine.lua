-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Prints a short banner to stdout when the module is loaded, showing the script path and current timestamp.
local function onModuleLoad()
  print("\n")
  print("========================================")
  print("CUSTOM COMBUSTION ENGINE SCRIPT LOADED!")
  print("File: lua/vehicle/powertrain/combustionEngine.lua")
  print("Time: " .. os.date())
  print("========================================")
  print("\n")
end

-- Call the initialization
onModuleLoad()

local M = {}

M.outputPorts = {[1] = true} --set dynamically
M.deviceCategories = {engine = true}

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

-- Builds torque and power curves for the given engine device, including variants for nitrous, turbo, supercharger and their combinations.
-- @param device The engine device (as produced by M.new) whose base torque curve and modifier submodules (turbocharger, supercharger, nitrousOxideInjection) will be used.
-- @return A table with:
--   maxRPM (number) — the engine's max RPM used for curve generation;
--   curves (array) — list of curve entries; each entry contains `torque` (table: indexed by RPM+1 -> torque), `power` (table: RPM+1 -> power), `name` (string), `priority` (number), `dash` (optional line-style) and `width` (line width);
--   maxTorque (number) — highest torque value across all generated curves;
--   maxPower (number) — highest power value across all generated curves;
--   maxTorqueRPM (number) — RPM index (stored as k+1) where maxTorque occurs;
--   maxPowerRPM (number) — RPM index (stored as k+1) where maxPower occurs;
--   finalCurveName (number) — placeholder/index of the final curve (set to 1 in this implementation);
--   deviceName (string) — device.name;
--   vehicleID (number) — id of the associated object (obj:getId()).
-- Notes:
--   - The function subtracts friction (static and dynamic, accounting for wear/damage) from base torque before computing power.
--   - Modifier curves are generated only if the corresponding submodule reports `isExisting`.
--   - Returned torque/power tables are keyed by (rpm_index + 1) to match the module's curve indexing convention.
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
  --scale torque ouput to some minimum, but do not let that minimum increase the actual scale (otherwise a min of 0.2 could "revive" an engine that sits at 0 scale already)
  device.outputTorqueState = max(device.outputTorqueState * state, min(maxReduction or 0, device.outputTorqueState))
  damageTracker.setDamage("engine", "engineReducedTorque", device.outputTorqueState < 1)
end

-- Disable the engine: cut output torque, mark disabled, stop starter sounds, and record damage.
-- This sets device.outputTorqueState to 0, device.isDisabled to true and clears starter engagement;
-- it also stops any active starter sound effects and records the "engineDisabled" damage state.
local function disable(device)
  device.outputTorqueState = 0
  device.isDisabled = true
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

-- Re-enable the engine after being disabled.
-- Ensures the engine can produce torque, re-enables starter control, clears active misfire state/timers,
-- and clears the recorded "engineDisabled" damage flag.
local function enable(device)
  device.outputTorqueState = 1
  device.isDisabled = false
  device.starterDisabled = false
  device.lastMisfireTime = 0
  device.misfireActive = false
  damageTracker.setDamage("engine", "engineDisabled", false)
end

-- Locks the engine permanently due to hydrolock/serious failure, disabling outputs and starter.
-- This sets the device into a broken/disabled state, zeros torque and angular-velocity outputs,
-- disables the starter, stops any active starter sounds, and records powertrain/engine damage.
-- @param device The engine device instance to lock up; its state fields (outputTorqueState, outputAVState,
--               isDisabled, isBroken, starterDisabled) will be modified and starter sounds stopped.
local function lockUp(device)
  device.outputTorqueState = 0
  device.outputAVState = 0
  device.isDisabled = true
  device.isBroken = true
  device.starterDisabled = true
  if device.starterEngagedCoef > 0 then
    device.starterEngagedCoef = 0
    obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
    end
  end
  damageTracker.setDamage("powertrain", device.name, true)
  damageTracker.setDamage("engine", "engineLockedUp", true)
end

-- Update engine and forced-induction sounds to match current RPM and load.
-- Computes smoothed RPM and engine-load, scales load for engine and exhaust channels,
-- converts to fundamental FMOD frequencies, and updates the configured engine/exhaust
-- sound instances. Also propagates sound updates to the turbocharger and supercharger.
-- @param device Engine device table. Must provide soundRPMSmoother, soundLoadSmoother,
--        outputAV1, torqueCurve, intakeAirDensityCoef, idleTorque, engineVolumeCoef,
--        soundMinLoadMix/soundMaxLoadMix, optional exhaust mix overrides, soundConfiguration,
--        and engineSoundID / engineSoundIDExhaust as applicable.
-- @param dt Delta time (seconds) for the smoothing updates.
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

-- Update engine flood/ hydrolock state based on immersion and RPM.
-- 
-- Checks whether the engine's water-damage nodes are submerged and, if flooding is possible,
-- increments or decrements device.floodLevel over time (dt, in seconds). Flooding and drying
-- rates are scaled by engine RPM (0 at 0 RPM, 1 at device.maxAV). When floodLevel exceeds
-- the module-level hydrolockThreshold the function marks the engine as hydrolocked, calls
-- device:lockup(), records the damage state, and emits a GUI message. The function also
-- updates a rounded flood-percentage (0–100) on change and emits appropriate flooding/drying
-- UI messages; it clears the hydrolocked damage when the percentage reaches 0.
-- 
-- Parameters:
-- @param device The engine device whose flood state is updated.
-- @param dt Time step in seconds used to advance the flood/dry integration.
-- 
-- Side effects:
-- - Mutates device.floodLevel and device.prevFloodPercent.
-- - May call device:lockup(), set damage states via damageTracker, and post GUI messages via guihooks.
local function checkHydroLocking(device, dt)
  -- Check if already hydrolocked
  if device.floodLevel > hydrolockThreshold then
    return
  end

  -- Check if engine can flood and all water damage nodes are underwater
  local isFlooding = device.canFlood
  for _, n in ipairs(device.waterDamageNodes) do
    isFlooding = isFlooding and obj:inWater(n)
    if not isFlooding then
      break
    end
  end

  -- Update damage tracker
  damageTracker.setDamage("engine", "engineIsHydrolocking", isFlooding)

  -- Calculate flooding/drying rates (now in 1% increments)
  local floodRate = 0.04  -- 2% per second when fully submerged and at max RPM
  local dryRate = -0.06   -- 3% per second when drying (slower than flooding)
  
  -- Scale rate by engine RPM (0% at 0 RPM, 100% at max RPM)
  local rpmFactor = min(1, abs(device.outputAV1) / device.maxAV)
  
  -- Apply rate based on flooding/drying state
  local rate = (isFlooding and floodRate or dryRate) * rpmFactor
  
  -- Update flood level with delta time
  device.floodLevel = min(1, max(0, device.floodLevel + rate * dt))
  
  -- Check for hydrolock condition
  if device.floodLevel > hydrolockThreshold then
    damageTracker.setDamage("engine", "engineHydrolocked", true)
    device:lockup()
    guihooks.message("vehicle.combustionEngine.engineHydrolocked", 4, "vehicle.damage.flood")
    return
  end

  -- Calculate current percentage (0-100)
  local currPercent = floor(device.floodLevel * 100 + 0.5)  -- Proper rounding to nearest integer
  
  -- Only update UI when percentage changes
  if currPercent ~= (device.prevFloodPercent or 0) then
    if currPercent > (device.prevFloodPercent or 0) then
      -- Flooding message
      guihooks.message({
        txt = "vehicle.combustionEngine.engineFlooding", 
        context = {percent = currPercent}
      }, 4, "vehicle.damage.flood")
    else
      -- Drying messages
      if currPercent <= 0 then
        damageTracker.setDamage("engine", "engineHydrolocked", false)
        guihooks.message("vehicle.combustionEngine.engineDried", 4, "vehicle.damage.flood")
      else
        guihooks.message({
          txt = "vehicle.combustionEngine.engineDrying", 
          context = {percent = currPercent}
        }, 4, "vehicle.damage.flood")
      end
    end
    device.prevFloodPercent = currPercent
  end
end

-- Update device.energyStorageRatios for each registered energy storage matching the device's required energy type.
-- For each matching storage, sets the ratio to 1 / device.storageWithEnergyCounter when that storage currently has storedEnergy > 0, otherwise sets it to 0.
-- Mutates device.energyStorageRatios in-place; ignores storages that are not found or whose energyType does not match.
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

-- Update engine graphics, audio, battery and per-frame visual/auxiliary state.
-- Performs non-physics, per-frame updates: manages stall/starter buzzers and starter sounds, updates fuel usage and output RPM, advances starter-related timers and ignition error timers, computes and updates the vehicle battery model (charge, voltage, brightness, electrical load), updates ignition error/coefficient smoothing, handles shut-off sound triggering, stall detection, rev/torque over-limit damage checks (may trigger lockup and UI messages), updates sound/exhaust parameters (compression brake, antilag), pushes fuel/exhaust delay lines, calls submodule GFX updates (turbo, supercharger, nitrous, thermals), computes intake air density, runs hydrolock checks, and clears per-frame fuel/energy accounting fields.
-- @param device The combustion engine device table to update (mutated in-place).
-- @param dt Delta time in seconds for this update step.
local function updateGFX(device, dt)

  if device.stallBuzzerSoundID then -- Check if the source was created successfully at init
    -- Condition: Ignition is ON, but engine RPM is below a threshold (e.g., 50% of idle)
    local shouldBuzzerBeActive = (device.ignitionCoef > 0) and (device.outputAV1 < device.starterMaxAV * 1.1)

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

  device:updateFuelUsage()

  device.outputRPM = device.outputAV1 * avToRPM

  device.starterThrottleKillTimer = max(device.starterThrottleKillTimer - dt, 0)
  device.lastStarterThrottleKillTimerEnd = max((device.lastStarterThrottleKillTimerEnd or 0) - os.clock()*1.5, 0)

  if device.starterEngagedCoef > 0 then
    -- if device.starterBattery then
    --   local starterSpentEnergy = 1 / guardZero(abs(device.outputAV1)) * dt * device.starterTorque / 0.5 --0.5 efficiency
    --   device.starterBattery.storedEnergy = device.starterBattery.storedEnergy - starterSpentEnergy
    -- --print(starterSpentEnergy)
    -- --print(device.starterBattery.remainingRatio)
    -- end

    -- device.starterThrottleKillCoef = 1-device.starterThrottleKillTimer / device.starterThrottleKillTimerStart + math.max(linearScale(device.starterThrottleKillTimer, device.starterThrottleKillTimerStart, 0, 0, 3), 0.2)-0.2


    local killCoefFac = 1
    if device.starterThrottleKillTimer > 0 then
      killCoefFac = 1 - device.starterThrottleKillTimer/device.starterThrottleKillTimerStart
      device.starterIgnitionErrorChance = killCoefFac*6*linearScale(device.thermals.engineBlockTemperature, -270, 15, 1, 0)
      killCoefFac = math.pow(killCoefFac, 8)*0.05
    end
    device.starterThrottleKillCoef = device.starterThrottleKillCoefSmoother:get(killCoefFac, dt)

    -- use lower starter max av multiplier in case the engine just doesnt start
    -- occasionally this would result in the engine starting and immediately shutting down, so its disabled
    local starterMaxAVMultiplier = 1.1 --math.min(1.1, device.outputAV1/device.starterMaxAV+(device.starterThrottleKillTimer == 0 and 0 or math.max(2.0, 1.1)))
    
    -- Initialize smoothed pitch value if not exists
    device.smoothedPitch = device.smoothedPitch or 0.5
    
    -- Calculate pitch with a more natural curve at low RPMs
    -- Use a logarithmic curve to make low RPMs sound more natural
    local rpmRatio = device.outputAV1 / (device.starterMaxAV * starterMaxAVMultiplier)
    
    -- Apply a logarithmic curve that's more natural for engine sounds
    -- This will make the pitch increase more slowly at lower RPMs
    local curvedRatio = math.log(1 + rpmRatio * 2) / math.log(3)
    
    -- Set pitch range for more natural sound
    local minPitch = 0.0   -- Lower minimum pitch for deeper sound at low RPM
    local maxPitch = 0.8   -- Slightly reduced max pitch for more realism
    
    -- Calculate final pitch with limits and apply a small offset to prevent extreme lows
    local targetPitch = minPitch + (maxPitch - minPitch) * curvedRatio
    
    -- Apply smoothing to prevent sudden pitch changes
    local smoothingFactor = 0.5
    device.smoothedPitch = device.smoothedPitch or targetPitch
    device.smoothedPitch = device.smoothedPitch + (targetPitch - device.smoothedPitch) * smoothingFactor
    
    -- Apply the smoothed pitch to the sounds
    obj:setPitch(device.engineMiscSounds.starterSoundEngine, device.smoothedPitch)
    if device.engineMiscSounds.starterSoundExhaust then
      obj:setPitch(device.engineMiscSounds.starterSoundExhaust, device.smoothedPitch)
    end

    if device.outputAV1 > device.starterMaxAV * 1.1 then
      device.starterThrottleKillTimer = 0
      device.starterEngagedCoef = 0
      device.starterThrottleKillCoef = 1
      device.starterThrottleKillCoefSmoother:set(device.starterThrottleKillCoef)
      device.starterDisabled = false
      device.starterIgnitionErrorChance = 0
      obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
      if device.engineMiscSounds.starterSoundExhaust then
        obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
      end
    end
  end

  -- Get current RPM
  local currentRPM = device.outputAV1 * avToRPM
  
  -- Update battery state
  local dt = dt or device.batteryUpdateFrequency or 1/60  -- Fixed timestep for battery updates
  
  -- Local function to initialize battery parameters
  local function initBattery(device, jbeamData)
    -- Set battery parameters based on system voltage (12V or 24V)
    local is24V = device.batterySystemVoltage == 24
    
    -- Set voltage thresholds based on system voltage
    device.batteryNominalVoltage = is24V and 27.6 or 13.8  -- 27.6V for 24V, 13.8V for 12V when fully charged
    device.batteryMinVoltage = is24V and 18.0 or 9.0       -- 18V for 24V, 9V for 12V systems
    device.batteryCutoffVoltage = is24V and 16.0 or 8.0    -- Absolute minimum voltage before complete cutoff
    device.batteryWarningVoltage = is24V and 22.0 or 11.0  -- Voltage when warning indicators activate
    device.batteryLowVoltage = is24V and 20.0 or 10.0      -- Voltage when systems start to fail
    
    -- Set charge and drain rates based on system voltage
    device.batteryChargeRate = is24V and 1.0 or 0.5       -- Higher charge rate for 24V systems
    device.batteryDrainRate = is24V and 30.0 or 15.0      -- Base drain rate when cranking (A)
    
    -- Get battery capacity from vehicle battery if available
    if electrics.values.batteryCapacity then
      device.batteryCapacity = electrics.values.batteryCapacity
    else
      -- Fallback to JBeam value or default (100Ah)
      device.batteryCapacity = jbeamData.batteryCapacity or 100.0
    end
  
    -- Initialize battery charge from vehicle state if available
    if electrics.values.batteryCharge then
      device.batteryCharge = electrics.values.batteryCharge
    else
      -- Start with full charge by default
      device.batteryCharge = 1.0
    end
  
    -- Log battery initialization
    log('I', 'combustionEngine.initBattery', 
        string.format('Battery initialized: %.1fV system, %.1fAh capacity', 
                      device.batterySystemVoltage, device.batteryCapacity))
  end

  -- Ensure battery parameters are initialized
  if not device.batteryNominalVoltage then
    -- Initialize battery if not already done
    local jbeamData = device.jbeamData or {}
    initBattery(device, jbeamData)
  end
  
  -- Update battery state based on engine and starter status
  local starterActive = device.starterEngagedCoef > 0
  local engineRunning = device.outputAV1 > device.starterMaxAV * 1.1
  
  -- Default values in case initialization failed
  device.batteryCharge = device.batteryCharge or 1.0
  device.batteryDrainScale = device.batteryDrainScale or 1.0
  
  if starterActive and not engineRunning then
    -- Drain battery when starting (higher drain for 24V systems)
    local drainRate = (device.batteryDrainRate or 15.0) * (device.batteryDrainScale or 1.0)
    device.batteryCharge = math.max(0, device.batteryCharge - (drainRate * dt) / ((device.batteryCapacity or 100.0) * 3600))
    device.batteryLoad = drainRate  -- Track current load in Amps
  elseif engineRunning then
    -- Recharge battery when engine is running above idle
    -- Charge rate is higher for 24V systems and scales with RPM
    local chargeRate = (device.batteryChargeRate or 0.5) * (device.outputAV1 / math.max(1, device.idleAV))
    device.batteryCharge = math.min(1.0, device.batteryCharge + (chargeRate * dt) / 3600)
    device.batteryLoad = -chargeRate  -- Negative load indicates charging
  else
    device.batteryLoad = 0  -- No load when engine is off and starter not engaged
  end
  
  -- Calculate battery voltage (scales with charge level using a curve)
  -- Use safe defaults if initialization failed
  local is24V = (device.batterySystemVoltage or 12) == 24
  local minVoltage = device.batteryMinVoltage or (is24V and 18.0 or 9.0)
  local maxVoltage = device.batteryNominalVoltage or (is24V and 27.6 or 13.8)
  local cutoffVoltage = device.batteryCutoffVoltage or (is24V and 16.0 or 8.0)
  
  -- Ensure we have valid values
  minVoltage = tonumber(minVoltage) or (is24V and 18.0 or 9.0)
  maxVoltage = tonumber(maxVoltage) or (is24V and 27.6 or 13.8)
  cutoffVoltage = tonumber(cutoffVoltage) or (is24V and 16.0 or 8.0)
  
  -- Ensure max > min
  if maxVoltage <= minVoltage then
    maxVoltage = minVoltage + (is24V and 10.0 or 5.0)
  end
  
  -- Calculate voltage with charge curve (more realistic than linear)

  local charge = math.max(0, math.min(1, device.batteryCharge or 1.0))
  local chargeCurve = math.pow(charge, 0.7)
  local baseVoltage = minVoltage + (maxVoltage - minVoltage) * chargeCurve

  -- Smoothly increase voltage toward nominal when engine is running and RPM increases
  local idleAV = device.idleAV or 50
  local maxAV = idleAV * 2.5 -- 1500rpm equivalent
  local rpm = device.outputAV1 or 0
  local engineRunning = rpm > idleAV * 1.1
  local nominalVoltage = maxVoltage
  local restingVoltage = baseVoltage
  if engineRunning and rpm > idleAV then
    local t = math.min((rpm - idleAV) / (maxAV - idleAV), 1)
    device.batteryVoltage = restingVoltage + (nominalVoltage - restingVoltage) * t
  else
    device.batteryVoltage = restingVoltage
  end

  -- Ensure voltage stays within bounds
  device.batteryVoltage = math.max(cutoffVoltage, math.min(maxVoltage, device.batteryVoltage))
  
  -- Calculate battery voltage factor for lights (0.5 to 1.0 range)
  -- Lights will start dimming below warning voltage
  local dimStartVoltage = device.batteryWarningVoltage or (is24V and 22.0 or 11.0)
  dimStartVoltage = tonumber(dimStartVoltage) or (is24V and 22.0 or 11.0)
  
  -- Ensure dimStartVoltage is between cutoff and max voltage
  dimStartVoltage = math.max(cutoffVoltage * 1.1, math.min(maxVoltage * 0.9, dimStartVoltage))
  
  -- Calculate full brightness voltage (slightly below nominal)
  local fullBrightnessVoltage = maxVoltage * 0.95  -- 95% of nominal
  
  -- Ensure fullBrightnessVoltage is above dimStartVoltage
  fullBrightnessVoltage = math.max(dimStartVoltage * 1.05, fullBrightnessVoltage)
  
  -- Calculate brightness factor with safety checks
  local batteryBrightnessFactor = linearScale(
    math.max(cutoffVoltage, math.min(maxVoltage, device.batteryVoltage)),
    dimStartVoltage,
    fullBrightnessVoltage,
    0.5,  -- Minimum brightness factor
    1.0   -- Maximum brightness factor
  )
  
  -- Update electrical system with current battery state
  if electrics.values then
    electrics.values.batteryVoltage = device.batteryVoltage
    electrics.values.batteryCharge = device.batteryCharge
  end
  batteryBrightnessFactor = math.max(0.2, math.min(1.0, batteryBrightnessFactor))  -- Clamp to 20-100%
  
  -- Base brightness based on RPM - starts dim and increases with RPM
  -- At 0 RPM: 0.4 (dim)
  -- At cranking RPM (200): ~0.5
  -- At idle (800): ~0.76
  -- At max RPM: 0.8
  local baseBrightness = linearScale(currentRPM, 0, device.maxRPM, 0.6, 1.0) * batteryBrightnessFactor
  
  -- Starter effect - when cranking, we want to see the brightness pulse with the engine
  -- This creates a flickering effect that gets faster as the engine speeds up
  local pulseEffect = 1.0
  if device.starterEngagedCoef > 0 then
    -- More noticeable pulsing during cranking
    local pulseFrequency = math.max(currentRPM * 0.05, 0.3)  -- Slower pulsing at low RPM
    local pulseAmplitude = 0.8  -- How much the brightness varies (smaller = less variation)
    local basePulse = math.sin(currentRPM * 0.1 * math.pi)
    pulseEffect = 1 + pulseAmplitude * basePulse
  end
  
  -- Combine effects
  -- When cranking, we want the brightness to be mostly controlled by RPM
  -- but with the pulsing effect on top
  local brightness = baseBrightness * pulseEffect
  
  -- Calculate electrical load coefficient based on battery state and brightness
  -- Lower battery voltage will reduce the electrical load coefficient more
  local loadWarnVoltage = device.batteryWarningVoltage * 0.85  -- Start warning slightly earlier for load reduction
  local loadMinVoltage = device.batteryLowVoltage * 0.75  -- Minimum voltage for load reduction
  
  -- Scale based on system voltage
  local batteryLoadFactor = linearScale(device.batteryVoltage, 
                                     loadMinVoltage, 
                                     loadWarnVoltage, 
                                     0.5, 1.0)
  batteryLoadFactor = math.max(0.5, math.min(1.0, batteryLoadFactor))  -- Clamp to 50-100%
  
  -- Apply battery load factor to brightness and ensure we stay within reasonable bounds
  electrics.values.electricalLoadCoef = math.min(math.max(brightness * batteryLoadFactor, 0.3), 1.0)
  
  -- Update battery drain scale based on electrical load (higher load = faster drain)
  device.batteryDrainScale = 0.5 + (electrics.values.electricalLoadCoef * 1.5)  -- 0.5x to 2.0x drain rate



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
    --device:lockUp()
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
  --update idle throttle
  device.idleTimer = device.idleTimer - dt
  if device.idleTimer <= 0 then
    local idleTimeRandomCoef = linearScale(device.idleTimeRandomness, 0, 1, 1, randomGauss3() * 0.6666667)
    device.idleTimer = device.idleTimer + device.idleTime * idleTimeRandomCoef
    -- device.idleTime
    local engineAV = device.outputAV1
    local highIdle = device.idleAV + math.max(math.min(60+linearScale(device.thermals.engineBlockTemperature, 60, -60, -60, 60), 250), 0)*0.6  -- ((max(-device.thermals.engineBlockTemperature, 10)-10) * 0.7)
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

-- Updates the engine's torque, rotational speed (AV), and related internal state for a single physics timestep.
-- This computes combustion/starter torque, friction, pumping/compression losses, battery/starter effects, per-cylinder
-- fuel/ignition/misfire behavior, flood handling, and updates engine output AV/torque, loads, and derived diagnostics.
-- It also advances the engine cycle position, updates ignition/misfire timers, adjusts battery charge/voltage while
-- cranking or charging, and runs the periodic fixed-step update when its timer elapses.
-- @param device The engine device table whose fields are read and written (torque curves, thermals, battery state,
--               cylinders, starter state, output ports, smoothing objects, and many diagnostics).
-- @param dt Time step in seconds for this update.
-- Note: This function has no return value; it mutates the provided device and sends side-effects (state updates,
-- periodic fixed-step invocation, and an inertial torque couple applied via the global simulation object).

local function updateTorque(device, dt)
  local isFlooded = device.floodLevel > 0.5  -- Adjust threshold as needed
  local isCranking = device.starterEngagedCoef > 0 and math.abs(device.outputAV1) < 300 * (math.pi/30)  -- Below 100 RPM

  local engineAV = device.outputAV1

  local throttle = (electrics.values[device.electricsThrottleName] or 0) * (electrics.values[device.electricsThrottleFactorName] or device.throttleFactor)

  local engineTempC = (device.thermals and device.thermals.engineBlockTemperature or 293.15) - 273.15  -- Default to 20°C if not available
  local engineTemp = engineTempC

  --don't include idle throttle as otherwise idle affects the turbo wastegate, do include it though if we have a raised idle throttle (eg semi truck hidh idle)
  device.requestedThrottle = max(throttle, device.idleAVOverwrite > 0 and device.idleThrottle or 0)

  throttle = min(max(max(device.idleThrottle, throttle) * (device.starterThrottleKillCoef+(1-device.starterIgnitionErrorCoef)*device.inertia*0.1) * device.ignitionCoef, 0), 1)

  throttle = device:applyRevLimiter(engineAV, throttle, dt)

  --smooth our actual throttle value to simulate various effects in a real engine that do not allow immediate throttle changes
  throttle = device.throttleSmoother:getUncapped(throttle, dt) --* 1.2
  local finalFriction = device.friction * device.wearFrictionCoef * device.damageFrictionCoef
  local finalDynamicFriction = device.dynamicFriction * device.wearDynamicFrictionCoef * device.damageDynamicFrictionCoef

  local tableRPM = floor(engineAV * avToRPM) or 0
  local torque = (device.torqueCurve[tableRPM] or 0) * device.intakeAirDensityCoef
  local maxCurrentTorque = torque - finalFriction - (finalDynamicFriction * engineAV)
  --blend pure throttle with the constant power map
  local throttleMap = smoothmin(max(throttle + throttle * device.maxPowerThrottleMap / (torque * device.forcedInductionCoef * engineAV + 1e-30) * (1 - throttle), 0), 1, (1 - throttle) * 0.8) --0.8 can be tweaked to adjust the peakiness of the throttlemap adjusted torque curve

  local ignitionCut = device.ignitionCutTime > 0
  torque = (torque * device.forcedInductionCoef * throttleMap) + device.nitrousOxideTorque
  torque = torque * device.outputTorqueState * (ignitionCut and 0 or 1) * device.slowIgnitionErrorCoef * device.fastIgnitionErrorCoef * device.starterIgnitionErrorCoef
  if device.maxTorqueLimit and device.maxTorqueLimit < math.huge then
    torque = min(torque, device.maxTorqueLimit)  -- Limit output torque to specified max
  end
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
  frictionTorque = min(frictionTorque, absEngineAV * device.inertia * 2000) * sign(engineAV)

  -- Initialize flood level tracking if needed
  device.floodLevel = device.floodLevel or 0
  
  -- Calculate starter torque with flood and wear factors
  local floodFactor = device.floodLevel * 0.006  -- Reduced from 0.7 to 0.3 for much less impact
  local starterWear = device.starterWear or 0
  local wearFactor = 1.0 - (starterWear * 0.8)  -- Reduced from 0.8 to 0.4 for less wear impact
  
  -- Calculate base starter torque based on engine size, type, and starter power
  local baseStarterTorque
 
    -- Check if this is a diesel engine based on fuel type
    local isDiesel = (device.requiredEnergyType == "diesel") or 
                     (device.engineType and (device.engineType == "diesel" or device.engineType == "dieselElectric"))

    -- Use device.starterTorque if explicitly set, otherwise use default values
    if device.starterTorque then
    -- Original torque values that were working
    if isDiesel then
      baseStarterTorque = --[[device.starterTorque or]] 180  -- Original value for diesel engines is 180
    else
      baseStarterTorque = --[[device.starterTorque or]] 210  -- Original value for gasoline engines is 110
    end
  end
  
  -- Base starter torque with reduced impact from flooding, wear, and temperature
  -- Reduce torque more significantly when flooded and in cold conditions
  local floodTorqueReduction = floodFactor * 0.8  -- Increased from 0.7
  local tempTorqueReduction = math.max(0, (0 - engineTempC) / 50)  -- Up to 40% reduction at -20°C
  baseStarterTorque = baseStarterTorque * (1 - floodTorqueReduction) * (1 - tempTorqueReduction) * wearFactor
  
  -- Calculate final starter torque with velocity-based reduction and engagement
  -- Multiplies starter engagement (0-1) by base torque, then scales based on engine speed
  
  -- Different torque curves for diesel vs gasoline engines
  local maxTorqueAtZeroRPM, minTorqueAtHighRPM
  
  -- Initialize compression and cylinder states if not exists
  device.compressionOscTimer = device.compressionOscTimer or 0
  device.compressionState = device.compressionState or 0
  device.compressionStateTimer = device.compressionStateTimer or 0
  
  -- Initialize per-cylinder fuel and combustion state
  if not device.cylinders then
    device.cylinders = {}
    local cylinderCount = device.fundamentalFrequencyCylinderCount or 8
    for i = 1, cylinderCount do
      device.cylinders[i] = {
        fuelAmount = 0,            -- Amount of fuel in cylinder (0-1)
        airAmount = 0.6,           -- Amount of air in cylinder (0-1)
        compressionRatio = 8,      -- Engine compression ratio
        isCompressing = false,     -- Whether cylinder is in compression stroke
        isFiring = false,          -- Whether cylinder is in power stroke
        sparkPlugFouled = false,   -- Whether spark plug is fouled (gasoline only)
        lastFired = -1,            -- Last cycle this cylinder fired
        misfireCount = 0,          -- Consecutive misfires
        temperature = 0,           -- Current temperature (for heat simulation)
        damage = 0,                -- Cylinder damage (0-1)
        lastFuelAddTime = -1       -- When was fuel last added (to prevent rapid adding)
      }
    end
  end
  

  
  -- Fuel injection parameters with temperature compensation
  local baseFuelAmount = 0.02  -- Base fuel scaled by enrichment
  local crankingFuelMultiplier = 1.5  -- Increased for better cold starts
  local maxFuelPerCylinder = 1.0
  local minFuelForInjection = 0.25  -- Slightly reduced minimum fuel for better cold starts
  
  -- Temperature-compensated combustion thresholds
  local tempAdjustment = math.max(0.3, 1.0 - (engineTempC / 100))  -- 0.3-1.0 based on temp
  local minFuelForCombustion = isCranking and (0.01 * tempAdjustment) or (0.1 * tempAdjustment)
  local minAirForCombustion = isCranking and (0.04 * tempAdjustment) or (0.4 * tempAdjustment)
  
  -- Ignition assistance during cranking - more help when cold
  local minIgnitionForCombustion = isCranking and 
    (0.15 * (1.5 - (engineTempC / 100 * 0.8))) or 0.5
  
  -- Enhanced compression oscillation with more realistic behavior
  -- Base oscillation speed scales with engine speed and cylinder count
  local oscSpeed = device.fundamentalFrequencyCylinderCount * 1.5
  
  -- More dynamic engine speed factor with better low-speed response
  local engineSpeedFactor = math.min(math.pow(math.abs(engineAV) * 0.9, 0.6), 300)
  
  -- Update oscillation timer with smooth acceleration/deceleration
  -- Initialize compression and cylinder states if not exists
  device.compressionOscTimer = device.compressionOscTimer or 0
  device.compressionState = device.compressionState or 0
  device.compressionStateTimer = device.compressionStateTimer or 0
  local targetOscSpeed = (engineSpeedFactor + oscSpeed) * 1.5
  device.compressionOscTimer = (device.compressionOscTimer + dt * targetOscSpeed) % (math.pi * 2)
  
  -- Create primary oscillation with sharper peaks and flatter valleys
  local oscFactor = math.sin(device.compressionOscTimer)
  local oscFactorSharp = math.pow(math.abs(oscFactor), 0.6) * (oscFactor >= 0 and 1 or -1)
  
  -- Add multiple harmonics for complex, realistic oscillation
  local oscFactor2 = math.sin(device.compressionOscTimer * 1.1) * 0.4  -- Slightly detuned for beating effect
  local oscFactor3 = math.sin(device.compressionOscTimer * 0.5) * 0.25  -- Sub-harmonic for low-end rumble
  
  -- Blend harmonics with emphasis on primary oscillation
  local combinedOscFactor = (oscFactorSharp * 0.7) + (oscFactor2 * 0.2) + (oscFactor3 * 0.1)
  
  -- Add compression pulses that align with cylinder firing order
  local compressionPulse = 0
  local cylinderCount = device.fundamentalFrequencyCylinderCount or 4  -- Default to 4 cylinders if not set
  local pulsePhase = (device.compressionOscTimer % (math.pi * 2 / cylinderCount)) / (math.pi * 2 / cylinderCount)
  
  -- Create more pronounced compression pulses with realistic timing
  if pulsePhase > 0.9 and pulsePhase < 1.1 then
    local pulseStrength = 0.8 + math.random() * 0.4  -- Random variation in pulse strength
    -- Shape the pulse with a smooth curve
    local pulseShape = math.sin((pulsePhase - 0.9) * (math.pi / 0.2) * 0.5)
    compressionPulse = pulseStrength * pulseShape * pulseShape
  end
  
  -- Combine base oscillation with compression pulses
  -- Apply temperature effect - more resistance when cold
  local normalizedTemp = math.max(0, math.min(1, (engineTempC + 20) / 100))  -- Normalize -20°C to 80°C to 0-1 range
  local tempEffect = 0.8 + (1 - normalizedTemp) * 0.6  -- 0.8-1.4 multiplier based on temperature
  combinedOscFactor = combinedOscFactor * 2.8 * tempEffect + compressionPulse * 1.2
  
  -- Apply a soft clip to prevent extreme values while maintaining peak shape
  combinedOscFactor = math.atan(combinedOscFactor * 0.5) * 1.5
  
  -- Get battery parameters
  local is24V = device.batterySystemVoltage == 24
  local minVoltage = is24V and 18.0 or 9.0  -- Minimum operating voltage under load
  local maxVoltage = is24V and 28.8 or 14.4  -- Maximum charging voltage
  local nominalVoltage = is24V and 24.0 or 12.0  -- Nominal system voltage
  
  -- Get current battery state (0.0 to 1.0)
  local chargeLevel = device.batteryCharge or 1.0
  
  -- Calculate open-circuit voltage (no load)
  local ocv = minVoltage + (maxVoltage - minVoltage) * math.pow(chargeLevel, 1.5)
  
  -- Battery charge/discharge logic
  local starterCurrent = 0
  local voltageDrop = 0
  local isEngineRunning = device.outputAV1 > device.starterMaxAV * 1.1
  
  -- Update battery charge based on current conditions
  if device.starterEngagedCoef > 0 then
    -- Base current draw (higher for 24V systems)
    local baseCurrent = is24V and 280 or 140  -- Amps
    
    -- Current increases with load (lower RPM = higher load)
    local loadFactor = 1.0 - math.min(1.0, math.abs(device.outputAV1) / (device.starterMaxAV * 0.7))
    starterCurrent = baseCurrent * (0.1 + 0.5 * loadFactor) * device.starterEngagedCoef
    
    -- Internal resistance (higher when battery is cold or discharged)
    local internalResistance = (is24V and 0.02 or 0.04) * (1.0 + (1.0 - chargeLevel) * 2.0)
    voltageDrop = starterCurrent * internalResistance    
    -- Calculate energy consumed by starter (in watt-seconds)
    local starterVoltage = ocv - voltageDrop
    local starterPower = starterCurrent * starterVoltage  -- Watts
    local energyConsumed = starterPower * dt  -- Watt-seconds
    
    -- Convert energy to battery charge (assuming 50Ah battery capacity)
    local batteryCapacity = 500 * 3600  -- 50Ah in watt-seconds (50A * 12V * 3600s)
    local chargeConsumed = energyConsumed / (batteryCapacity * (is24V and 2 or 1))
    
    -- Update battery charge
    if device.starterEngagedCoef > 0 then
      device.batteryCharge = math.max(0, device.batteryCharge - chargeConsumed)
    end
  elseif isEngineRunning and device.starterEngagedCoef == 0 then
    -- Charge battery when engine is running and starter is off
    local chargeRate = dt * 0.001  -- Base charge rate per second
    device.batteryCharge = math.min(1, device.batteryCharge + chargeRate)
  end
  
  -- Calculate actual battery voltage under load
  local batteryVoltage = math.max(minVoltage * 0.8, ocv - voltageDrop)
  
  -- Calculate voltage factor (0.0 to 1.0) for torque calculation
  local batteryVoltageFactor = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage)
  batteryVoltageFactor = math.max(0, math.min(1, batteryVoltageFactor))  -- Clamp to 0-1 range
  
  -- Apply non-linear response curve (more sensitive at lower voltages)
  batteryVoltageFactor = math.pow(batteryVoltageFactor, is24V and 1.5 or 0.6)
  
  -- Set minimum voltage factor to prevent complete loss of starter torque
  local minVoltageFactor = is24V and 0.15 or 0.1
  batteryVoltageFactor = math.max(minVoltageFactor, batteryVoltageFactor)
  
  -- Store the calculated battery voltage for other systems
  device.batteryVoltage = batteryVoltage
  
  -- Enhanced battery state logging with debug info
  if not device.batteryDebugInitialized then
    print("[Battery] Initializing battery debug logging")
    device.batteryLogCounter = 0
    device.batteryDebugInitialized = true
  end
  
  -- Always increment counter
  device.batteryLogCounter = device.batteryLogCounter + 1

  --change to true to enable debugging logs
  local debugBatt = true

  if debugBatt then 
  --log detailed battery state every 50 physics ticks when starter is engaged, or every 200 ticks when not
    if device.starterEngagedCoef > 0 or isEngineRunning or device.batteryLogCounter % 1000 == 0 then
      if device.batteryLogCounter % 1000 == 0 then
        local currentVoltage = batteryVoltage
        local starterTorque = (isDiesel and 8.95 or 7.86) * device.starterMaxAV * (tempEffect or 1.0) * (voltageEffect or 1.0)
      
        -- Debug print to console
        print(string.format(
          "[Battery] Debug - starterEngagedCoef: %.2f, voltageFactor: %.2f, starterCurrent: %.2f",
          device.starterEngagedCoef,
          batteryVoltageFactor,
          starterCurrent
        ))
      
        -- Detailed GUI message
        -- Calculate additional battery metrics
        local stateOfCharge = math.max(0, math.min(1, (batteryVoltage - minVoltage) / (maxVoltage - minVoltage)))
        local starterPower = starterCurrent * batteryVoltage / 1000  -- In kW
      
        local logMsg = string.format(
          "Battery State:\n" ..
          "Voltage: %.1fV (%.0f%% SOC)\n" ..
          "System: %s | Charge: %.0f%%\n" ..
         "Current: %.1fA | Power: %.1fkW\n" ..
          "Temp Effect: %.2fx | Load: %.0f%%\n" ..
          "Starter Torque: %.1f Nm | RPM: %.0f\n" ..
          "Counter: %d | Time: %.1fs",
          batteryVoltage,
          stateOfCharge * 100,
          is24V and "24V" or "12V",
          chargeLevel * 100,
          starterCurrent,
          starterPower,
          tempEffect or 1.0,
          (1.0 - (device.outputAV1 / (device.starterMaxAV * 0.5))) * 100,
          starterTorque,
          device.outputAV1 * 9.5493,  -- Convert rad/s to RPM
          device.batteryLogCounter,
          device.batteryLogCounter * 0.0167  -- Approximate time in seconds (60 ticks per second)
        )

          -- Send to both console and GUI for visibility
          print("[Battery] " .. logMsg:gsub("\n", " | "))
          
          -- Show battery status message when voltage is low or when starter is engaged
          local currentVoltage = device.batteryNominalVoltage * device.batteryCharge
          if currentVoltage < device.batteryWarningVoltage or device.starterEngagedCoef > 0 then
            local battStatus = string.format(
              "Battery Status:\n" ..
              "Voltage: %.1fV / %.1fV\n" ..
              "Charge: %d%%\n" ..
              "Starter: %s",
              currentVoltage,
              device.batteryNominalVoltage,
              math.floor(device.batteryCharge * 100),
              device.starterEngagedCoef > 0 and "ENGAGED" or "DISENGAGED"
            )
            
            if currentVoltage < device.batteryLowVoltage then
              battStatus = battStatus .. "\n\nWARNING: Low battery voltage!"
              if device.starterEngagedCoef > 0 then
                battStatus = battStatus .. "\nEngine may not start!"
              end
            end
            
            gui.message(battStatus, 2.0, "debug")
          end
          
          -- Original debug message (commented out)
          -- gui.message({logMsg, "vehicle.debug"}, 1.0, "debug")
      end
    else
      -- Log when starter is not engaged (less frequently)
      if device.batteryLogCounter % 1000 == 0 then
        print("[Battery] Starter not engaged (coef: " .. tostring(device.starterEngagedCoef) .. ")")
      end
    end
  end
  
  -- Enhanced starter torque calculation with better battery and temperature modeling
  -- Current configuration (as of 2024-07-15):
  -- Diesel:
  --   Base torque: 8.95 * starterMaxAV
  --   Voltage effect: pow(batteryVoltageFactor, 0.8)
  --   Temp effect: 1.2 + (engineTemp * 1.2)
  --   Compression effect: 0.8-1.3 based on temperature
  --   Min torque at high RPM: 0.128 * invStarterMaxAV
  -- Gasoline:
  --   Base torque: 7.66 * starterMaxAV
  --   Voltage effect: pow(batteryVoltageFactor, 0.7)
  --   Temp effect: 1.4 + (engineTemp * 0.4)
  --   Compression effect: 0.8-1.3 based on temperature
  --   Min torque at high RPM: 0.016 * invStarterMaxAV
  if isDiesel then
    -- Diesel engines - high compression requires more torque
    local baseTorque = device.starterTorque * 0.2 or device.starterMaxAV * 2.95  -- Increased base torque for diesel
    
    -- More aggressive battery voltage scaling - diesels need more power
    local voltageEffect = math.pow(batteryVoltageFactor, 0.5)  -- More sensitive to voltage drops
    baseTorque = baseTorque * (0.05 + 0.95 * voltageEffect)
    
    -- Temperature effect - much harder to start when cold
    local tempEffect = 0.4 + (engineTemp * 0.2)  -- 0.8-1.2 multiplier based on temperature
    baseTorque = baseTorque * tempEffect
    
    -- Strong oscillation to simulate high compression
    -- More pronounced when cold
    local compressionEffect = 0.1 + (1 - engineTemp) * 0.5  -- 0.8-1.2 multiplier
    maxTorqueAtZeroRPM = baseTorque * (1 + (combinedOscFactor * 0.2 * compressionEffect))
    
    -- Minimum torque at high RPM - ensure engine keeps spinning
    minTorqueAtHighRPM = device.invStarterMaxAV * 0.028 * voltageEffect * tempEffect
  else
    -- Gasoline engines - less compression, easier starting
    local baseTorque = device.starterTorque -- * 1.45 or device.starterMaxAV * 18.26  -- Increased base torque for better cranking
    
    -- Battery voltage scaling
    local voltageEffect = math.pow(batteryVoltageFactor, 1.1)
    baseTorque = baseTorque * (0.62 + 0.95 * voltageEffect)
    
    -- Temperature effect - still significant but less than diesel
    local normalizedTemp = math.max(0, math.min(1, (engineTempC + 20) / 100))  -- Normalize -20°C to 80°C to 0-1 range
    local tempEffect = 0.9 + (normalizedTemp * 0.4)  -- 0.8-1.2 multiplier based on temperature
    baseTorque = baseTorque * tempEffect
    
    -- Moderate oscillation for compression
    local compressionEffect = 0.4 + ((1 - normalizedTemp) * 0.3)  -- 0.8-1.1 multiplier
    maxTorqueAtZeroRPM = baseTorque * (1 + (combinedOscFactor * 0.4 * compressionEffect))
    
    -- Minimum torque at high RPM
    minTorqueAtHighRPM = device.invStarterMaxAV * 0.076 * voltageEffect * tempEffect
  end
  
  -- Apply the torque curve based on engine speed
  -- As engine speeds up, torque decreases linearly from maxTorqueAtZeroRPM to minTorqueAtHighRPM
  -- Ensure we always apply torque in the positive direction during cranking
  local engineSpeedFactor = math.max(0, 0.9 - math.abs(engineAV) * device.invStarterMaxAV)
  local baseStarterTorque = device.starterEngagedCoef * maxTorqueAtZeroRPM * engineSpeedFactor

  -- Apply compression stroke simulation and ignition errors
  if device.starterEngagedCoef > 0 then
    -- Initialize ignition error timers if not set
    device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer or 0
    device.fastIgnitionErrorTimer = device.fastIgnitionErrorTimer or 0
    device.starterIgnitionErrorTimer = device.starterIgnitionErrorTimer or 0
    
    -- Initialize ignition error states if not set
    device.slowIgnitionErrorActive = device.slowIgnitionErrorActive or false
    device.fastIgnitionErrorActive = device.fastIgnitionErrorActive or false
    device.starterIgnitionErrorActive = device.starterIgnitionErrorActive or false
    
    -- Initialize ignition error coefficients if not set
    device.slowIgnitionErrorCoef = device.slowIgnitionErrorCoef or 1
    device.fastIgnitionErrorCoef = device.fastIgnitionErrorCoef or 1
    device.starterIgnitionErrorCoef = device.starterIgnitionErrorCoef or 1
    
    -- Initialize ignition error durations if not set
    device.slowIgnitionErrorDuration = device.slowIgnitionErrorDuration or 0
    device.fastIgnitionErrorDuration = device.fastIgnitionErrorDuration or 0
    device.starterIgnitionErrorDuration = device.starterIgnitionErrorDuration or 0
    
    -- Get number of cylinders from JBEAM
    local cylinderCount = device.fundamentalFrequencyCylinderCount or 8
    
    -- Track engine cycle position
    device.cyclePosition = device.cyclePosition or 0
    
    -- Calculate cycle advancement based purely on angular velocity
    -- Each cylinder fires once per 2 revolutions (4-stroke cycle)
    -- So for each radian the engine turns, we advance the cycle by 1/(4*pi) per cylinder
    local radiansPerCycle = 4 * math.pi  -- 2 revolutions = 4*pi radians
    local cycleAdvance = (math.abs(device.outputAV1) * dt) / radiansPerCycle * cylinderCount
    
    -- Update cylinder states based on cycle position
    local cyclePosPerCylinder = 2 * math.pi / cylinderCount
    local currentCylinder = math.floor((device.cyclePosition % (2 * math.pi)) / cyclePosPerCylinder) + 1
    
    -- Update each cylinder's state based on its position in the cycle
    for i = 1, cylinderCount do
      local cylinder = device.cylinders[i]
      local cylinderAngle = (i - 1) * cyclePosPerCylinder
      local cyclePos = (device.cyclePosition - cylinderAngle) % (2 * math.pi)
      
      -- Determine stroke (0-3: intake, compression, power, exhaust)
      local stroke = math.floor(cyclePos / (math.pi / 2)) % 4
      
      -- Update cylinder state based on stroke
      cylinder.isCompressing = (stroke == 1)
      cylinder.isFiring = (stroke == 2)
      
      -- Handle fuel injection during intake stroke
      if stroke == 0 and not cylinder.isFiring and throttle > 0.1 then
        -- Check if we should be adding fuel
        local timeSinceLastFuel = (device.cyclePosition - (cylinder.lastFuelAddTime or -10))
        local isReadyForFuel = timeSinceLastFuel > (isCranking and 0.3 or 0.2)  -- More frequent injection when cranking
        
        if (isStarting or isRunning) and isReadyForFuel then
          -- Calculate base fuel amount with all enrichment factors
          local fuelAmount = baseFuelAmount * throttle
          
          -- Apply cranking enrichment when cranking
          if isCranking then
            fuelAmount = fuelAmount * crankingFuelMultiplier
            
            -- Add extra fuel pulse at the beginning of cranking
            if device.cyclePosition < (2 * math.pi) then  -- First revolution
              fuelAmount = fuelAmount * 0.15
            end
          end
          
          -- Apply choke enrichment
          fuelAmount = fuelAmount * (0.10 + (chokeEffect * 0.5))  -- Up to 50% extra fuel with choke
          
          -- Get cylinder-specific flood level
          local cylinderFlood = device.cylinderFloodLevels[i] or 0
          
          -- Reduce or cut fuel if cylinder is flooded
          if cylinderFlood > 0.7 then
            -- Complete fuel cut for severely flooded cylinder
            fuelAmount = 0
            
            -- Small chance to clear some flood when fuel is cut
            if math.random() < 0.1 then
              device.cylinderFloodLevels[i] = math.max(0, cylinderFlood - 0.1)
            end
          elseif cylinderFlood > 0.3 then
            -- Progressive fuel reduction for partially flooded cylinder
            fuelAmount = fuelAmount * (1.0 - cylinderFlood)
          end
          
          -- Ensure minimum fuel injection amount
          fuelAmount = math.max(fuelAmount, minFuelForInjection)
          
          -- Add fuel to cylinder with a minimum amount
          local newFuel = math.min(maxFuelPerCylinder, cylinder.fuelAmount + fuelAmount * dt * 80)
          
          local debugFuel = true
          
          -- Debug output for fuel addition with rate limiting
          if debugFuel and fuelAmount > 0 then
            device.lastFuelLogTime = device.lastFuelLogTime or {}
            device.lastFuelLogTime[i] = device.lastFuelLogTime[i] or 0
            local currentTime = os.clock()
            
            if currentTime - device.lastFuelLogTime[i] > 1.0 then  -- Limit to once per second per cylinder
              print(string.format("[FUEL] Cyl %d: Adding %.6f (total: %.6f) at pos %.2f, RPM: %.1f, Throttle: %.2f, State: %s, Temp: %.2f (%.1fs)", 
                i, fuelAmount, newFuel, device.cyclePosition, 
                math.abs(device.outputAV1) * (30/math.pi),  -- Convert rad/s to RPM
                throttle,
                isCranking and "CRANKING" or (isRunning and "RUNNING" or "STARTING"),
                device.temperature or 0,
                currentTime))
              device.lastFuelLogTime[i] = currentTime
            end
          end
          
          cylinder.fuelAmount = newFuel
          cylinder.airAmount = 1.0  -- Reset air amount
          cylinder.lastFuelAddTime = device.cyclePosition
        end
      end
      
      -- Handle compression stroke
      if cylinder.isCompressing and not cylinder.isFiring then
        -- Increase temperature due to compression
        local compressionHeat = 0.1 * (cylinder.compressionRatio ^ 0.3) * dt * 60
        cylinder.temperature = math.min(1.0, cylinder.temperature + compressionHeat)
        
        -- Check for pre-ignition (knock)
        if cylinder.temperature > 0.8 and math.random() < 0.1 then
          -- Simulate knock effect
          device.knockLevel = (device.knockLevel or 0) + 0.2
          if device.knockLevel > 1.0 then
            -- Severe knock - reduce power
            torque = torque * 0.9
          end
        end
      end
      
      -- Handle power stroke
      if cylinder.isFiring then
        -- Enhanced misfire effects with temperature-dependent severity
        if cylinder.fuelAmount > minFuelForCombustion * 0.8 then  -- Slightly more forgiving fuel threshold
          -- Much stronger temperature-based severity - peaks at -20°C and below
          local tempSeverity = math.min(1.0, math.max(0, (20 - engineTempC) / 20))
          
          -- Base torque reduction (30-80% of starter torque)
          local baseReduction = 0.8 + (tempSeverity * 0.5)
          
          -- Add randomness to severity (0.8x to 1.2x)
          local randomFactor = 0.8 + (math.random() * 0.4)
          local misfireTorque = -device.starterTorque * baseReduction * randomFactor
          
          -- Stronger oscillation based on temperature and RPM
          local rpmFactor = math.min(1.0, math.abs(device.outputAV1) / (device.idleAV * 0.7))
          local oscillation = math.sin(device.cyclePosition * device.fundamentalFrequencyCylinderCount) * 
                            (0.3 + (tempSeverity * 0.7)) *  -- More oscillation when cold
                            (1.0 - (rpmFactor * 0.8)) *     -- Less oscillation at higher RPM
                            device.starterTorque
        end
        
        -- Adjust combustion thresholds based on temperature (easier to ignite when warmer)
        local tempAdjustedMinFuel = minFuelForCombustion * (1.0 + (1.0 - engineTemp) * 0.5)  -- Higher threshold when colder
        local tempAdjustedMinAir = minAirForCombustion * (1.0 - (1.0 - engineTemp) * 0.2)  -- Slightly lower when colder
        
        -- Check for combustion conditions with temperature compensation
        local hasEnoughFuel = cylinder.fuelAmount >= tempAdjustedMinFuel
        local hasEnoughAir = cylinder.airAmount >= tempAdjustedMinAir
        local hasEnoughIgnition = device.ignitionCoef >= (minIgnitionForCombustion * (1.0 + (1.0 - engineTemp) * 0.3))  -- More lenient when colder
        
        if hasEnoughFuel and hasEnoughAir and hasEnoughIgnition then
          -- Successful combustion - more sensitive to mixture when cold
          local tempFactor = device.thermals and (device.thermals.engineBlockTemperature / 100) or 0.5
          tempFactor = math.max(0.3, math.min(1.0, tempFactor))  -- Clamp between 0.3 and 1.0
          
          -- Adjust efficiency based on temperature and mixture
          local combustionEfficiency = math.min(cylinder.fuelAmount, cylinder.airAmount) * (0.7 + tempFactor * 0.3)
          local power = combustionEfficiency * (0.5 + tempFactor * 0.5) * 1.2  -- Less aggressive power curve
          
          -- Add power to engine
          torque = torque * (1 + power)
          
          -- Consume fuel and air, being very conservative during cranking
          local fuelConsumption = isCranking and 0.02 or 0.1  -- Much less fuel consumed during cranking
          local minFuelToKeep = isCranking and 0.005 or 0.01  -- Keep less fuel in cylinder during cranking
          
          cylinder.fuelAmount = math.max(minFuelToKeep, cylinder.fuelAmount - fuelConsumption)
          cylinder.airAmount = math.max(0.02, cylinder.airAmount - (isCranking and 0.05 or 0.1))
          
          -- Debug output for successful combustion with rate limiting
          if debugFuel then
            device.lastCombustLogTime = device.lastCombustLogTime or {}
            device.lastCombustLogTime[i] = device.lastCombustLogTime[i] or 0
            local currentTime = os.clock()
            
            if currentTime - device.lastCombustLogTime[i] > 0.5 then  -- Limit to twice per second per cylinder
              print(string.format("[COMBUST] Cyl %d: Success! Fuel: %.4f, Air: %.4f, Temp: %.2f (%.1fs)", 
                i, cylinder.fuelAmount, cylinder.airAmount, cylinder.temperature, currentTime))
              device.lastCombustLogTime[i] = currentTime
            end
          end
          
          -- Reset misfire counter
          cylinder.misfireCount = 0
          cylinder.lastFired = device.cyclePosition
        else
          -- Misfire - no combustion
          cylinder.misfireCount = (cylinder.misfireCount or 0) + 1
          
          -- Debug output for misfire with rate limiting
          if debugFuel then
            device.lastMisfireLogTime = device.lastMisfireLogTime or {}
            device.lastMisfireLogTime[i] = device.lastMisfireLogTime[i] or 0
            local currentTime = os.clock()
            
            if currentTime - device.lastMisfireLogTime[i] > 0.5 then  -- Limit to twice per second per cylinder
              print(string.format("[MISFIRE] Cyl %d: #%d - Fuel: %.6f/%.6f, Air: %.3f/%.3f, Ign: %.2f/%.2f, Temp: %.2f, State: %s, Throttle: %.2f, RPM: %.1f (%.1fs)", 
                i, 
                cylinder.misfireCount, 
                cylinder.fuelAmount, tempAdjustedMinFuel,
                cylinder.airAmount, tempAdjustedMinAir,
                device.ignitionCoef or 0, minIgnitionForCombustion, 
                device.temperature or 0,
                isCranking and "CRANKING" or (isRunning and "RUNNING" or "STARTING"),
                throttle,
                math.abs(device.outputAV1) * (30/math.pi),
                currentTime))
              device.lastMisfireLogTime[i] = currentTime
            end
          end
          
          -- Add some resistance for misfire, but only if we have fuel (actual misfire)
          if cylinder.fuelAmount > minFuelForCombustion * 0.1 and cylinder.misfireCount > 1 then  -- After 2 consecutive misfires with fuel
            torque = torque * 0.59  -- Very slight power loss
            
            -- Only increase flood level if we actually have fuel to flood with
            -- Make flood level increase more gradual and dependent on fuel amount
            if isCranking and device.floodLevel < 1.0 and cylinder.misfireCount > 5 and cylinder.fuelAmount > minFuelForCombustion then
              local floodIncrement = 0.005 * (1.0 + cylinder.fuelAmount * 0.005)  -- 0.5% to 1.5% increase based on fuel
              device.floodLevel = math.min(1.0, (device.floodLevel or 0) + floodIncrement)
              if debugFuel then
                device.lastFloodIncLogTime = device.lastFloodIncLogTime or {}
                device.lastFloodIncLogTime[i] = device.lastFloodIncLogTime[i] or 0
                local currentTime = os.clock()
                
                if currentTime - device.lastFloodIncLogTime[i] > 0.5 then  -- Limit to twice per second per cylinder
                  print(string.format("[FLOOD] Cyl %d: Level %.3f (+%.3f), Misfires: %d (%.1fs)", 
                    i, device.floodLevel, floodIncrement, cylinder.misfireCount, currentTime))
                  device.lastFloodIncLogTime[i] = currentTime
                end
              end
            end
          end
          
          -- Clear some fuel on misfire to prevent flooding
          if cylinder.fuelAmount > 0.05 then
            cylinder.fuelAmount = cylinder.fuelAmount * 0.8
          end
        end
        
        -- Reset temperature after power stroke
        cylinder.temperature = 0
      end
    end
    
    -- Apply a very small minimum advance only when engine is nearly stopped
    -- This helps with initial movement without making it progress too fast
    if math.abs(device.outputAV1) < 0.3 then  -- ~1 RPM
      cycleAdvance = math.max(cycleAdvance, 0.25 * dt * 60)  -- Very small advance
    end
    
    -- Update cycle position (2 revolutions per 4-stroke cycle)
    device.cyclePosition = (device.cyclePosition + cycleAdvance) % (cylinderCount * 2)
    
    -- Calculate stroke position (0-1)
    local strokePos = device.cyclePosition / (cylinderCount * 2)
    
    -- Calculate stroke effect
    local strokeEffect = 1
    local strokeAmplitude = 0.9  -- Increased amplitude for more visible pulsing
    
    -- Calculate stroke phase (0-1)
    local strokePhase = math.floor(strokePos * cylinderCount * 2) % 2  -- 0 for compression, 1 for power
    
    -- Calculate RPM for effect scaling (clamped to reasonable range)
    local rpm = math.abs(device.outputAV1) * avToRPM
    local rpmFactor = math.min(1, rpm / 1000)  -- 1.0 at 1000 RPM, tapers off above
    
    -- Stronger effect at lower RPMs, tapers off as RPM increases
    local compressionStrength = 1.8 * (1 - rpmFactor * 0.8)  -- 1.5x at 0 RPM, 0.3x at 1000 RPM
    local powerStrength = 3.0 * (1 - rpmFactor * 0.8)        -- 2.0x at 0 RPM, 0.2x at 1000 RPM
    
    -- Calculate ignition error chances based on RPM and engine state
    local isDiesel = device.requiredEnergyType == "diesel"
    local engineBlockTemp = device.thermals and device.thermals.engineBlockTemperature or 20  -- Default to room temp if not available
    local coldStartFactor = engineBlockTemp < 20 and 1.5 or 1.0  -- More likely to misfire when cold
    
    -- Slow ignition errors (more likely at lower RPMs)
    device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer - dt
    if device.slowIgnitionErrorTimer <= 0 then
      device.slowIgnitionErrorTimer = math.random(device.slowIgnitionErrorInterval) * 0.1
      local slowIgnitionChance = (1 - rpmFactor) * 0.2 * coldStartFactor  -- 20% chance at 0 RPM, 0% at max RPM
      if math.random() < slowIgnitionChance then
        device.slowIgnitionErrorActive = true
        device.slowIgnitionErrorDuration = 0.2 + math.random() * 0.3  -- Random duration between 0.2 and 0.5 seconds
      else
        device.slowIgnitionErrorActive = false
      end
    end
    
    -- Fast ignition errors (more likely at higher RPMs)
    device.fastIgnitionErrorTimer = device.fastIgnitionErrorTimer - dt
    if device.fastIgnitionErrorTimer <= 0 then
      device.fastIgnitionErrorTimer = math.random() * 0.1  -- Random interval between 0 and 0.1 seconds
      local fastIgnitionChance = rpmFactor * 0.15 * coldStartFactor  -- 0% chance at 0 RPM, 15% at max RPM
      if math.random() < fastIgnitionChance then
        device.fastIgnitionErrorActive = true
        device.fastIgnitionErrorDuration = 0.1 + math.random() * 0.1  -- Random duration between 0.1 and 0.2 seconds
      else
        device.fastIgnitionErrorActive = false
      end
    end
    
    -- Starter ignition errors (more likely when cold)
    device.starterIgnitionErrorTimer = device.starterIgnitionErrorTimer - dt
    if device.starterIgnitionErrorTimer <= 0 then
      device.starterIgnitionErrorTimer = math.random() * 0.2  -- Random interval between 0 and 0.2 seconds
      local starterIgnitionChance = coldStartFactor * 0.1  -- 10% chance when cold, 5% when warm
      if math.random() < starterIgnitionChance then
        device.starterIgnitionErrorActive = true
        device.starterIgnitionErrorDuration = 0.2 + math.random() * 0.2  -- Random duration between 0.2 and 0.4 seconds
      else
        device.starterIgnitionErrorActive = false
      end
    end
    
    -- Update error coefficients
    if device.slowIgnitionErrorActive then
      device.slowIgnitionErrorDuration = device.slowIgnitionErrorDuration - dt
      device.slowIgnitionErrorCoef = 0.5  -- Reduce torque by 50% during slow ignition error
      if device.slowIgnitionErrorDuration <= 0 then
        device.slowIgnitionErrorActive = false
        device.slowIgnitionErrorCoef = 1
      end
    end
    
    if device.fastIgnitionErrorActive then
      device.fastIgnitionErrorDuration = device.fastIgnitionErrorDuration - dt
      device.fastIgnitionErrorCoef = 0.7  -- Reduce torque by 30% during fast ignition error
      if device.fastIgnitionErrorDuration <= 0 then
        device.fastIgnitionErrorActive = false
        device.fastIgnitionErrorCoef = 1
      end
    end
    
    if device.starterIgnitionErrorActive then
      device.starterIgnitionErrorDuration = device.starterIgnitionErrorDuration - dt
      device.starterIgnitionErrorCoef = 0.6  -- Reduce torque by 40% during starter ignition error
      if device.starterIgnitionErrorDuration <= 0 then
        device.starterIgnitionErrorActive = false
      end
    end
    
    -- Stronger effect at lower RPMs, tapers off as RPM increases
    local compressionStrength = 3.8 * (1 - rpmFactor * 0.1) * tempEffect
    local powerStrength = 1.0 * (1 - rpmFactor * 0.1)
    
    -- Calculate stroke effect with phase-specific timing
    local phaseOffset = strokePhase == 0 and 0 or 0.25
    local phaseSine = math.sin((strokePos + phaseOffset) * math.pi)
    
    -- Apply effects with RPM-based scaling
    if strokePhase == 0 then
      -- Compression stroke - resistance increases as piston moves up
      strokeEffect = strokeAmplitude * (1 - phaseSine) * compressionStrength
      
      -- Additional resistance when cold
      if engineTempC < 20 then
        local coldResistance = (20 - engineTempC) / 20  -- 0-1 based on how cold
        strokeEffect = strokeEffect * (1.0 + coldResistance * 2.0)  -- Up to 3x more resistance when very cold
      end
    else
      -- Power stroke - push decreases as piston moves down
      -- Reduced power when cold
      local coldPowerReduction = math.max(0.3, 1.0 - ((60 - math.min(60, engineTempC)) / 60) * 0.7)  -- Down to 30% power when very cold
      strokeEffect = strokeAmplitude * (phaseSine - 1) * powerStrength * coldPowerReduction
    end
    
    -- Update stroke log time without logging
    device.lastStrokeLogTime = currentTime
    
    -- Apply stroke effect to torque with a minimum threshold
    local minStarterTorque = baseStarterTorque * 2.4  -- Reduced minimum to 70% for more visible pulsing
    local modifiedTorque = max(baseStarterTorque * (1 - strokeEffect), minStarterTorque)
    
    -- Smooth the transition between strokes
    device.lastModifiedTorque = device.lastModifiedTorque or baseStarterTorque
    starterTorque = device.lastModifiedTorque + (modifiedTorque - device.lastModifiedTorque) * 0.9  -- 20% smoothing
    device.lastModifiedTorque = starterTorque
  end
  
  -- Misfire severity is now handled inside the cylinder loop
  
  -- Enhanced misfire timer and conditions with temperature sensitivity
  if device.isMisfiring then
    device.misfireTimer = device.misfireTimer - dt
    
    -- Make misfires last longer when cold
    local tempFactor = math.max(0, (20 - engineTempC) / 40)  -- 0 at 20°C, 0.5 at 0°C, 1.0 at -20°C
    local timeDilation = 1.0 + (tempFactor * 2.0)  -- 1x to 3x longer duration when cold
    
    if device.misfireTimer <= 0 then
      device.isMisfiring = false
      device.misfireTorque = 0
    else
      -- Make the torque reduction pulse slightly for more noticeable effect
      local pulse = 1.0 + (math.sin(device.misfireTimer * 20) * 0.3)  -- 0.7x to 1.3x pulsing
      starterTorque = starterTorque - (device.misfireTorque * pulse)
      
      -- Add some random variation to make it feel more mechanical
      if math.random() < 0.1 then  -- 10% chance per update to add a small random kick
        starterTorque = starterTorque + ((math.random() - 0.5) * device.starterTorque * 0.2)
      end
    end
  end
  
  -- Initialize or update engine coast down state
  if device.starterEngagedCoef == 0 and not device.starterEngagedCoef == 1 then
    device.coastDownRPM = device.outputAV1 * avToRPM
    device.coastDownTime = 0
  end
  device.lastStarterEngagedCoef = device.starterEngagedCoef

  --iterate over all connected clutches and sum their torqueDiff to know the final torque load on the engine
  local torqueDiffSum = 0
  for i = 1, device.activeOutputPortCount do
    local outputPort = device.activeOutputPorts[i]
    torqueDiffSum = torqueDiffSum + device.clutchChildren[outputPort].torqueDiff
  end
  --calculate the AV based on all loads
  local outputAV = (engineAV + dt * (torque - torqueDiffSum - frictionTorque - compressionBrakeTorque + baseStarterTorque) * device.invEngInertia) * device.outputAVState
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
  elseif subSystem == "supercharger" then
    local integrityValue, integrityState = device.supercharger.getPartCondition()
    return integrityValue, integrityState
  end
end

local function validate(device)
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

-- Activate the starter sequence for the given engine device.
-- 
-- Updates starter-related coefficients and timers, adjusts ignition behavior based on
-- engine block temperature and fuel availability, and triggers enable/disable and damage
-- markers as appropriate. Also starts/stops starter sound effects and records the
-- starter as engaged (sets device.starterEngagedCoef = 1).
--
-- Side effects:
-- - Modifies device.ignitionCoef, device.starterThrottleKillTimer, device.starterIgnitionErrorTimer,
--   device.starterIgnitionErrorChance, device.starterIgnitionErrorCoef, device.starterThrottleKillTimerStart,
--   device.starterEngagedCoef and device.engineMiscSounds.loopTimer.
-- - May call enable(device) or disable(device) depending on temperature/fuel.
-- - May set or clear damageTracker flags for engineDisabled.
-- - Plays/cuts starter SFX (starterSoundEngine, optional starterSoundExhaust).
--
-- Behavior notes:
-- - Uses a larger cold-start time coefficient for diesel devices than for gasoline.
-- - If engineBlockTemperature is extremely low (<= -270°C) the engine is disabled and a GUI message is emitted.
-- - For cold but not extreme temperatures, starter timers and ignition error parameters are scaled based on temperature.
-- - If temperature is warm (>= 16°C) related slow-ignition and idle-read error values are cleared.
local function activateStarter(device)
  device.ignitionCoef = device.ignitionCoef * 1.5
  if device.starterEngagedCoef ~= 1 then
    device.starterThrottleKillCoef = 0
    local coldBlockStartTimeCoef = device.requiredEnergyType == "diesel" and 36 or 16
    if device.lastStarterThrottleKillTimerEnd and device.lastStarterThrottleKillTimerEnd > 2.6 then
      device.starterThrottleKillTimer = device.lastStarterThrottleKillTimerEnd or device.starterThrottleKillTime
    elseif device.thermals.engineBlockTemperature <= -270 then
      device.ignitionCoef = device.ignitionCoef * 1
      disable(device)
      damageTracker.setDamage("engine", "engineDisabled", false)
    -- damageTracker.setDamage("engine", "EngineTooColdToStart", true)
      gui.message("Engine too cold to start!\n" .. 
                  "Engine block temperature: " .. device.thermals.engineBlockTemperature .. "°C\n" .. 
                  "You can try to start the engine\n" .. 
                  "But I doubt it will work")
    elseif device.thermals.engineBlockTemperature >= -270 and device.thermals.engineBlockTemperature < 25 and device.hasFuel then
      enable(device)
      damageTracker.setDamage("engine", "engineDisabled", false)
      -- device.ignitionCoef = device.ignitionCoef * 2
    -- damageTracker.setDamage("engine", "EngineTooColdToStart", false)
      device.starterThrottleKillTimer = device.starterThrottleKillTime * linearScale(device.thermals.engineBlockTemperature, -60, 5, coldBlockStartTimeCoef, 2.70)
      device.starterIgnitionErrorTimer = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)        -- Longer error duration when cold
      device.starterIgnitionErrorChance = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)     -- 70% max chance at -270°C
      device.starterIgnitionErrorCoef = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)         -- Stronger effect when cold
     -- device.idleAVReadError = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)             -- More pronounced RPM fluctuations
     -- device.idleAVReadErrorChance = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)       -- 70% max chance at -270°C
     -- device.idleAVReadErrorCoef = linearScale(device.thermals.engineBlockTemperature, -270, 15, coldBlockStartTimeCoef, 0.1)     -- Stronger effect when cold
    elseif device.thermals.engineBlockTemperature >= 16 then
      device.slowIgnitionErrorTimer = 0
      device.slowIgnitionErrorChance = 0
      device.slowIgnitionErrorCoef = 0
      device.idleAVReadError = 0
      device.idleAVReadErrorChance = 0
      device.idleAVReadErrorCoef = 0
    end

    device.starterThrottleKillTimerStart = device.starterThrottleKillTimer
    device.starterEngagedCoef = 1

    obj:cutSFX(device.engineMiscSounds.starterSoundEngine)
    obj:playSFX(device.engineMiscSounds.starterSoundEngine)

    if device.engineMiscSounds.starterSoundExhaust then
      obj:cutSFX(device.engineMiscSounds.starterSoundExhaust)
      obj:playSFX(device.engineMiscSounds.starterSoundExhaust)
    end

    device.engineMiscSounds.loopTimer = device.engineMiscSounds.loopTime
  end
end

local function cutIgnition(device, time)
  device.ignitionCutTime = time
end

-- Deactivates the starter after a crank attempt and applies the appropriate throttle/ignition state.
-- If the engine reached running speed (device.outputAV1 > device.starterMaxAV * 1.1) the starter is treated as
-- having successfully started the engine; otherwise it is treated as a failed start.
--
-- Side effects:
-- - Updates device.lastStarterThrottleKillTimerEnd and clears device.starterThrottleKillTimer.
-- - Sets device.starterEngagedCoef to 0 and device.starterIgnitionErrorChance to 0.
-- - Sets device.starterThrottleKillCoef to 1 when the engine started, or 0 when it did not; writes this value
--   into device.starterThrottleKillCoefSmoother.
-- - Stops starter SFX on a successful start (stopSFX) or abruptly cuts them on failure (cutSFX). Also handles
--   exhaust starter sound if present.
--
-- @param device The engine device table whose starter state and sounds are to be deactivated.
local function deactivateStarter(device)
  --if we happen to crank barely long enough, then do allow the engine to start up, otherwise, we stay with the throttle kill coef as is (usually at 0)
  local didStart = false
  if device.outputAV1 > device.starterMaxAV * 1.1 then
    device.starterThrottleKillCoef = 0.45
    didStart = true
  end

  device.lastStarterThrottleKillTimerEnd = device.starterThrottleKillTimer
  device.starterThrottleKillTimer = 0
  device.starterEngagedCoef = 0
  device.starterThrottleKillCoef = didStart and 1 or 0
  device.starterThrottleKillCoefSmoother:set(device.starterThrottleKillCoef)

  device.starterIgnitionErrorChance = 0
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

-- Sets the engine ignition state.
-- When `value` > 0 the ignition coefficient is set to 1 (on); otherwise it is set to 0 (off).
-- If ignition is turned off (`value == 0`), the starter throttle kill timer and starter engaged coefficient are cleared.
-- Turning ignition off will also request the engine shut-off sound if the current output AV exceeds `starterMaxAV * 1.1`.
-- @param value Numeric ignition input; >0 turns ignition on, 0 turns it off.
local function setIgnition(device, value)
  device.ignitionCoef = value > 0 and 1 or 0
  if value == 0 then
    device.starterThrottleKillTimer = 0
    device.starterEngagedCoef = 0
    if device.outputAV1 > device.starterMaxAV * 1.1 then
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

-- Reset and configure engine and subsystem sounds according to the provided JBEAM sound configuration.
-- If a new sound config is present, this resets RPM/load smoothers, disables legacy sounds, and updates
-- engine/exhaust sound parameters (nodes, gain, muffling). Logs an error if the named sound config is missing.
-- Always invokes resetSounds on turbocharger, supercharger, nitrous injection, and thermals to reinitialize their audio state.
-- @param device The engine device instance whose sound state will be reset and reconfigured.
-- @param jbeamData Table of JBEAM configuration data; expects keys referencing sound/turbo/supercharger/nitrous configs.
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
        local maxExhaustAudioOpennessCoef = 0.5
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
  device.maxTorqueLimit = jbeamData.maxTorqueLimit or device.maxTorqueLimit or math.huge

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

  device.floodLevel = 0
  device.prevFloodPercent = 0

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

  device.overRevDamage = 0
  device.overTorqueDamage = 0

  -- Initialize battery parameters
  if not device.batterySystemVoltage then
    -- Default to 12V for gasoline, 24V for diesel if not set
    device.batterySystemVoltage = (jbeamData.requiredEnergyType == "diesel" or jbeamData.engineType == "diesel") and 24 or 12
  end
  
  -- Initialize battery charge if not set
  device.batteryCharge = device.batteryCharge or 1.0
  device.batteryDrainScale = device.batteryDrainScale or 1.0
  device.batteryLoad = 0

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

local function initBattery(device, jbeamData)
  -- Set battery parameters based on system voltage (12V or 24V)
  local is24V = device.batterySystemVoltage == 24
  
  -- Set voltage thresholds based on system voltage
  device.batteryNominalVoltage = is24V and 27.6 or 13.8  -- 27.6V for 24V, 13.8V for 12V when fully charged
  device.batteryMinVoltage = is24V and 18.0 or 9.0       -- 18V for 24V, 9V for 12V systems
  device.batteryCutoffVoltage = is24V and 16.0 or 8.0    -- Absolute minimum voltage before complete cutoff
  device.batteryWarningVoltage = is24V and 22.0 or 11.0  -- Voltage when warning indicators activate
  device.batteryLowVoltage = is24V and 20.0 or 10.0      -- Voltage when systems start to fail
  
  -- Set charge and drain rates based on system voltage
  device.batteryChargeRate = is24V and 1.0 or 0.5       -- Higher charge rate for 24V systems
  device.batteryDrainRate = is24V and 30.0 or 15.0      -- Base drain rate when cranking (A)
  
  -- Get battery capacity from vehicle battery if available
  if electrics.values.batteryCapacity then
    device.batteryCapacity = electrics.values.batteryCapacity
  else
    -- Fallback to JBeam value or default (100Ah)
    device.batteryCapacity = jbeamData.batteryCapacity or 100.0
  end
  
  -- Initialize battery charge from vehicle state if available
  if electrics.values.batteryCharge then
    device.batteryCharge = electrics.values.batteryCharge
  else
    -- Start with full charge by default
    device.batteryCharge = 1.0
  end
  
  -- Log battery initialization
  log('I', 'combustionEngine.initBattery', 
      string.format('Battery initialized: %.1fV system, %.1fAh capacity', 
                    device.batterySystemVoltage, device.batteryCapacity))
end

-- Initialize and configure engine and exhaust sound sources and related audio parameters on the device.
--
-- This function creates/startup SFX sources (starter, shut-off, stall buzzer, intake/exhaust blended sounds),
-- applies volumes and EQ/muffling parameters, initializes engine/exhaust sound parameter lists and smoothers,
-- and delegates sound initialization to turbocharger, supercharger, nitrous, and thermals subsystems.
-- It will also switch the device to use the new sound update callback when applicable and log missing configs.
--
-- @param jbeamData Table of JBEAM-provided sound configuration and sample keys used by this engine:
--        expected keys include (but are not limited to) starterSample, starterSampleExhaust,
--        shutOffSampleEngine, shutOffSampleExhaust, starterVolume*, shutOffVolume*,
--        soundConfig, soundConfigExhaust, rpmSmootherInRate, rpmSmootherOutRate,
--        loadSmootherInRate, loadSmootherOutRate, turbocharger, supercharger, nitrousOxideInjection.
--        Missing or invalid soundConfig entries are logged; presence of soundConfig determines whether
--        the new blended sound pipeline is initialized.
local function initSounds(device, jbeamData)
  local exhaustEndNodes = device.thermals.exhaustEndNodes or {}

  device.engineMiscSounds = {
    starterSoundEngine = obj:createSFXSource2(jbeamData.starterSample or "event:>Engine>Starter>Old_V2", "AudioDefaultLoop3D", "", device.engineNodeID, 0),
    starterVolume = jbeamData.starterVolume or 1,
    starterVolumeExhaust = jbeamData.starterVolumeExhaust or 1,
    shutOffVolumeEngine = jbeamData.shutOffVolumeEngine or 1,
    shutOffVolumeExhaust = jbeamData.shutOffVolumeExhaust or 1
  }
  obj:setVolume(device.engineMiscSounds.starterSoundEngine, device.engineMiscSounds.starterVolume)
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
  
  --[[Initialize misfire sound
  if jbeamData.misfireSample then
    device.misfireSoundID = obj:createSFXSource2(jbeamData.misfireSample, "AudioDefault3D", "", device.engineNodeID, 0)
    device.misfireSoundID:setVolume(0.8)
  end]]
  device.supercharger.initSounds(v.data[jbeamData.supercharger])
  device.nitrousOxideInjection.initSounds(v.data[jbeamData.nitrousOxideInjection])
  device.thermals.initSounds(jbeamData)
end

-- Create and initialize a combustion engine device from JBeam data and return the device table.
-- 
-- The constructor builds a fully configured engine object ready for registration with the powertrain system:
-- - Initializes battery/electrical defaults (auto-selects 12V/24V from fuel type unless overridden).
-- - Configures inertial, starter, idle, ignition, rev-limiter and damage-related defaults.
-- - Builds torque and compression-brake curves from provided torque tables and applies redline drop-off.
-- - Initializes subsystems: thermals, turbocharger, supercharger and nitrous oxide injection (if defined).
-- - Prepares output ports, torque reaction nodes, water-damage/flooding support, and energy storage bookkeeping.
-- - Resets relevant damage tracker flags and selects update callbacks.
-- 
-- @param jbeamData Table of engine configuration values parsed from JBeam (examples: requiredEnergyType, torque, idleRPM, maxRPM, batterySystemVoltage, turbocharger, supercharger, nitrousOxideInjection, torqueCompressionBrake, revLimiter settings, etc.). Fields are read extensively; unspecified values use sensible defaults.
-- @return device A table representing the initialized combustion engine device (contains state, subsystem objects, methods, and data structures used by the simulation).
local function new(jbeamData)
  -- Create device table with basic battery parameters
  local isDiesel = (jbeamData.requiredEnergyType == "diesel") or (jbeamData.engineType == "diesel")
  local device = {
    -- Battery simulation - automatically detect voltage based on engine type
    -- 24V for diesel, 12V for gasoline by default (can be overridden in JBeam)
    isDieselEngine = isDiesel,
    
    -- Basic battery parameters (will be fully initialized by initBattery)
    batteryCharge = 1.0,  -- Will be updated by initBattery
    batterySystemVoltage = jbeamData.batterySystemVoltage or (isDiesel and 24 or 12),  -- Auto-detect based on engine type
    batteryCapacity = 100.0,  -- Will be updated by initBattery
    batteryLoad = 0.0,  -- Current load in A
    batteryDrainScale = 1.0,  -- Scale factor for battery drain
    
    -- Device categories and other properties
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
    maxTorqueLimit = jbeamData.maxTorqueLimit or math.huge,  -- Allow override from JBeam, default to no limit
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
    starterTorque = jbeamData.starterTorque or (jbeamData.friction * 15),
    starterMaxAV = (jbeamData.starterMaxRPM or jbeamData.idleRPM * 0.7) * rpmToAV,
    starterTorqueMultiplier = jbeamData.starterTorqueMultiplier or 1,
    shutOffSoundRequested = false,
    starterEngagedCoef = 0,
    starterThrottleKillCoef = 1,
    starterThrottleKillCoefSmoother = newTemporalSmoothing(70, 40),
    starterThrottleKillTimer = 0,
    starterThrottleKillTimerStart = 0,
    starterThrottleKillTime = jbeamData.starterThrottleKillTime or 1.5,
    starterDisabled = false,
    stallTimer = 1,
    isStalled = false,
    floodLevel = 0,
    prevFloodPercent = 0,
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
    stallBuzzerSample = jbeamData.stallBuzzerSample or "lua/vehicle/powertrain/stall_buzzer.wav", -- Default path adjusted
    stallBuzzerVolume = jbeamData.stallBuzzerVolume or 0.5, -- also tied to "OTHER" volume slider in options
    stallBuzzerCrankingPitch = jbeamData.stallBuzzerCrankingPitch or 0.3,
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
    print (string.format("RPM = %5.0f, TORQUE = %4.0f", v.rpm, v.torque))
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
  
  -- Initialize misfire tracking variables
  device.misfireTimer = 0
  device.misfireActive = false
  device.misfireTorque = 0
  device.misfireDuration = 0
  
  -- Initialize compression stroke variables
  device.fundamentalFrequencyCylinderCount = jbeamData.fundamentalFrequencyCylinderCount or 8
  device.cyclePosition = 0
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

M.new = new

local command = "obj:queueGameEngineLua(string.format('scenarios.getScenario().wheelDataCallback(%s)', serialize({wheels.wheels[0].absActive, wheels.wheels[0].angularVelocity, wheels.wheels[0].angularVelocityBrakeCouple})))"

return M

