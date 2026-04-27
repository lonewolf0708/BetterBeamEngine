-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
-- Module-level initialization
local function onModuleLoad()
	print("\n")
	print("========================================")
	print("CUSTOM COMBUSTION ENGINE SCRIPT LOADED!")
	print("VER. 2.1 by BetaWolf00708")
	print("File: lua/vehicle/powertrain/combustionEngine.lua")
	print("Time: " .. os.date())
	print("========================================")
	print("\n")
end

local M = {}

M.outputPorts = { [1] = true } -- set dynamically
M.deviceCategories = { engine = true }

local delayLine = rerequire("delayLine")
local carburetorModule = rerequire("lua/vehicle/powertrain/carburetor")
local chokeModule = rerequire("lua/vehicle/powertrain/choking")

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

local torqueDebug = false
local hesitationDebug = true
local starterDebug = false
local debugBatt = false

local fuelProperties = {
	gasoline = { -- Default alias for 91
		category = "gasoline",
		octaneRating = 91,
		energyDensity = 1.0,
		volatility = 1.0,
		ignitionTemp = 280,
		description = "Standard Gasoline (91)",
	},
	gasoline91 = {
		category = "gasoline",
		octaneRating = 91,
		energyDensity = 1.0,
		volatility = 1.0,
		ignitionTemp = 280,
		description = "Unleaded 91",
	},
	gasoline95 = {
		category = "gasoline",
		octaneRating = 95,
		energyDensity = 1.025,
		volatility = 1.05,
		ignitionTemp = 300,
		description = "Premium 95",
	},
	gasoline98 = {
		category = "gasoline",
		octaneRating = 98,
		energyDensity = 1.05,
		volatility = 1.1,
		ignitionTemp = 320,
		description = "Super 98",
	},
	diesel = {
		category = "diesel",
		octaneRating = 45, -- Average Cetane
		energyDensity = 1.15,
		volatility = 0.6,
		ignitionTemp = 210,
		description = "Standard Diesel",
	},
	biodiesel = {
		category = "diesel",
		octaneRating = 48,
		energyDensity = 1.08,
		volatility = 0.5,
		ignitionTemp = 230,
		description = "Biodiesel",
	},
	ethanol = { -- Alias for E85
		category = "ethanol",
		octaneRating = 105,
		energyDensity = 0.75,
		volatility = 1.2,
		ignitionTemp = 380,
		description = "E85 Ethanol",
	},
	e85 = {
		category = "ethanol",
		octaneRating = 105,
		energyDensity = 0.75,
		volatility = 1.2,
		ignitionTemp = 380,
		description = "E85 Ethanol",
	},
}

--[[local function checkFuelCompatibility(device)
	if not device.energyStorage then
		return device.requiredEnergyType -- Default
	end

	local storage = energyStorage.getStorage(device.energyStorage)
	if not storage then
		return device.energyStorage
	end

	local fuelType = storage.energyType or "gasoline"
	local properties = fuelProperties[fuelType] or device.energyStorage

	if fuelType ~= device.requiredEnergyType then
		device.fuelIncompatible = true
		if (device.lastFuelWarningTime or 0) + 10 < (device.time or 0) then
			guihooks.trigger("Message", {
				string.format(
					"FUEL INCOMPATIBILITY: %s in %s engine!",
					properties.description,
					device.requiredEnergyType
				),
				5,
				"danger",
			})
			device.lastFuelWarningTime = device.time
		end
	end

	return properties
end]]

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
			torqueCurve[k + 1] = v
				- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
				- (
					device.dynamicFriction
					* device.wearDynamicFrictionCoef
					* device.damageDynamicFrictionCoef
					* k
					* rpmToAV
				)
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

	table.insert(curves, curveCounter, {
		torque = torqueCurve,
		power = powerCurve,
		name = "NA",
		priority = 10,
	})

	if device.nitrousOxideInjection.isExisting then
		local torqueCurveNitrous = {}
		local powerCurveNitrous = {}
		nitrousTorques = device.nitrousOxideInjection.getAddedTorque()

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveNitrous[k + 1] = v
					+ (nitrousTorques[k] or 0)
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveNitrous,
			power = powerCurveNitrous,
			name = "N2O",
			priority = 20,
		})
	end

	if device.turbocharger.isExisting then
		local torqueCurveTurbo = {}
		local powerCurveTurbo = {}
		turboCoefs = device.turbocharger.getTorqueCoefs()

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveTurbo[k + 1] = (v * (turboCoefs[k] or 0))
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveTurbo,
			power = powerCurveTurbo,
			name = "Turbo",
			priority = 30,
		})
	end

	if device.supercharger.isExisting then
		local torqueCurveSupercharger = {}
		local powerCurveSupercharger = {}
		superchargerCoefs = device.supercharger.getTorqueCoefs()

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveSupercharger[k + 1] = (v * (superchargerCoefs[k] or 0))
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveSupercharger,
			power = powerCurveSupercharger,
			name = "SC",
			priority = 40,
		})
	end

	if device.turbocharger.isExisting and device.supercharger.isExisting then
		local torqueCurveFinal = {}
		local powerCurveFinal = {}

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveFinal[k + 1] = (v * (turboCoefs[k] or 0) * (superchargerCoefs[k] or 0))
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveFinal,
			power = powerCurveFinal,
			name = "Turbo + SC",
			priority = 50,
		})
	end

	if device.turbocharger.isExisting and device.nitrousOxideInjection.isExisting then
		local torqueCurveFinal = {}
		local powerCurveFinal = {}

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveFinal[k + 1] = (v * (turboCoefs[k] or 0) + (nitrousTorques[k] or 0))
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveFinal,
			power = powerCurveFinal,
			name = "Turbo + N2O",
			priority = 60,
		})
	end

	if device.supercharger.isExisting and device.nitrousOxideInjection.isExisting then
		local torqueCurveFinal = {}
		local powerCurveFinal = {}

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveFinal[k + 1] = (v * (superchargerCoefs[k] or 0) + (nitrousTorques[k] or 0))
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveFinal,
			power = powerCurveFinal,
			name = "SC + N2O",
			priority = 70,
		})
	end

	if
		device.turbocharger.isExisting
		and device.supercharger.isExisting
		and device.nitrousOxideInjection.isExisting
	then
		local torqueCurveFinal = {}
		local powerCurveFinal = {}

		for k, v in pairs(device.torqueCurve) do
			if type(k) == "number" and k < maxRPM then
				torqueCurveFinal[k + 1] = (
					v * (turboCoefs[k] or 0) * (superchargerCoefs[k] or 0) + (nitrousTorques[k] or 0)
				)
					- device.friction * device.wearFrictionCoef * device.damageFrictionCoef
					- (
						device.dynamicFriction
						* device.wearDynamicFrictionCoef
						* device.damageDynamicFrictionCoef
						* k
						* rpmToAV
					)
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
		table.insert(curves, curveCounter, {
			torque = torqueCurveFinal,
			power = powerCurveFinal,
			name = "Turbo + SC + N2O",
			priority = 80,
		})
	end

	table.sort(curves, function(a, b)
		local ra, rb = a.priority, b.priority
		if ra == rb then
			return a.name < b.name
		else
			return ra > rb
		end
	end)

	local dashes = { nil, { 10, 4 }, { 8, 3, 4, 3 }, { 6, 3, 2, 3 }, { 5, 3 } }
	for k, v in ipairs(curves) do
		v.dash = dashes[k]
		v.width = 2
	end

	return {
		maxRPM = maxRPM,
		curves = curves,
		maxTorque = maxTorque,
		maxPower = maxPower,
		maxTorqueRPM = maxTorqueRPM,
		maxPowerRPM = maxPowerRPM,
		finalCurveName = 1,
		deviceName = device.name,
		vehicleID = obj:getId(),
	}
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
	-- scale torque ouput to some minimum, but do not let that minimum increase the actual scale (otherwise a min of 0.2 could "revive" an engine that sits at 0 scale already)
	device.outputTorqueState = max(device.outputTorqueState * state, min(maxReduction or 0, device.outputTorqueState))
	damageTracker.setDamage("engine", "engineReducedTorque", device.outputTorqueState < 1)
end

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

local function enable(device)
	device.outputTorqueState = 1
	device.isDisabled = false
	device.starterDisabled = false
	device.lastMisfireTime = 0
	device.misfireActive = false
	damageTracker.setDamage("engine", "engineDisabled", false)
end

local function lockUp(device)
	device.outputTorqueState = 0
	device.outputAVState = 0
	device.isDisabled = true
	device.isBroken = true
	device.starterDisabled = false
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

local function updateSounds(device, dt)
	local rpm = device.soundRPMSmoother:get(abs(device.outputAV1 * avToRPM), dt)
	local maxCurrentTorque = (device.torqueCurve[floor(rpm)] or 1) * device.intakeAirDensityCoef
	local engineLoad = device.soundLoadSmoother:get(device.instantEngineLoad, dt)
	local baseLoad = 0.3 * min(device.idleTorque / maxCurrentTorque, 1)
	engineLoad = max(engineLoad - baseLoad, 0) / (1 - baseLoad)
	local volumeCoef = rpm > 0.1 and device.engineVolumeCoef or 0

	if device.engineSoundID then
		local scaledEngineLoad = engineLoad * (device.soundMaxLoadMix - device.soundMinLoadMix) + device.soundMinLoadMix
		local fundamentalFreq =
			sounds.hzToFMODHz(rpm * device.soundConfiguration.engine.params.fundamentalFrequencyRPMCoef)
		obj:setEngineSound(device.engineSoundID, rpm, scaledEngineLoad, fundamentalFreq, volumeCoef)
	end

	if device.engineSoundIDExhaust then
		local minLoad = device.soundMinLoadMixExhaust or device.soundMinLoadMix
		local scaledEngineLoadExhaust = engineLoad
				* ((device.soundMaxLoadMixExhaust or device.soundMaxLoadMix) - minLoad)
			+ minLoad
		local fundamentalFreqExhaust =
			sounds.hzToFMODHz(rpm * device.soundConfiguration.exhaust.params.fundamentalFrequencyRPMCoef)
		obj:setEngineSound(
			device.engineSoundIDExhaust,
			rpm,
			scaledEngineLoadExhaust,
			fundamentalFreqExhaust,
			volumeCoef
		)
	end

	device.turbocharger.updateSounds()
	device.supercharger.updateSounds()
end

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
	local floodRate = 0.02 -- 4% per second when fully submerged and at max RPM
	local dryRate = -0.04 -- 2% per second when drying (slower than flooding)

	-- Scale rate by engine RPM (0% at 0 RPM, 100% at max RPM)
	local rpmFactor = min(1, abs(device.outputAV1) / device.maxAV)

	-- Apply rate based on flooding/drying state
	local rate = (isFlooding and floodRate or dryRate) * rpmFactor

	-- Update flood level with delta time
	device.floodLevel = min(1, max(0, device.floodLevel + rate * dt))

	-- Check for hydrolock condition
	if device.floodLevel > hydrolockThreshold then
		damageTracker.setDamage("engine", "engineHydrolocked", true)
		device:lockUp()
		guihooks.message("vehicle.combustionEngine.engineHydrolocked", 4, "vehicle.damage.flood")
		return
	end

	-- Calculate current percentage (0-100)
	local currPercent = floor(device.floodLevel * 100 + 0.5) -- Proper rounding to nearest integer

	-- Only update UI when percentage changes
	if currPercent ~= (device.prevFloodPercent or 0) then
		if currPercent > (device.prevFloodPercent or 0) then
			-- Flooding message
			guihooks.message({
				txt = "vehicle.combustionEngine.engineFlooding",
				context = { percent = currPercent },
			}, 4, "vehicle.damage.flood")
		else
			-- Drying messages
			if currPercent <= 0 then
				damageTracker.setDamage("engine", "engineHydrolocked", false)
				guihooks.message("vehicle.combustionEngine.engineDried", 4, "vehicle.damage.flood")
			else
				guihooks.message({
					txt = "vehicle.combustionEngine.engineDrying",
					context = { percent = currPercent },
				}, 4, "vehicle.damage.flood")
			end
		end
		device.prevFloodPercent = currPercent
	end
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
			storage.storedEnergy =
				max(storage.storedEnergy - (device.spentEnergy * device.energyStorageRatios[storage.name]), 0)
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
	device.lastStarterThrottleKillTimerEnd = max((device.lastStarterThrottleKillTimerEnd or 0) - dt * 0.5, 0)

	if device.starterEngagedCoef > 0 then
		-- Initialize hesitation timer if not already set
		if not device.starterThrottleKillTimerStart or device.starterThrottleKillTimerStart == 0 then
			-- Scale hesitation period with temperature - colder = longer cranking time
			local baseHesitationTime = 12.0 -- Base time at normal temps (20°C)
			local engineTempC = (device.thermals and device.thermals.engineBlockTemperature) or 20
			-- Temp factor: 0 at 20°C, increases as temp drops, max 1.5x at -20°C or below
			local tempFactor = math.max(0, math.min(1.5, (20 - engineTempC) / 40))
			device.starterThrottleKillTimerStart = baseHesitationTime * (1 + tempFactor)
			device.starterThrottleKillTimer = device.starterThrottleKillTimerStart
		end

		if device.starterBattery then
			local starterSpentEnergy = 1 / guardZero(abs(device.outputAV1)) * dt * device.starterTorque / 1.5 -- 0.5 efficiency
			device.starterBattery.storedEnergy = device.starterBattery.storedEnergy - starterSpentEnergy
		end

		device.starterThrottleKillCoef = 1
			- device.starterThrottleKillTimer / device.starterThrottleKillTimerStart
			+ math.max(linearScale(device.starterThrottleKillTimer, device.starterThrottleKillTimerStart, 0, 0, 3), 0.2)
			- 0.2

		local killCoefFac = 1
		if device.starterThrottleKillTimer > 0 then
			killCoefFac = 1 - device.starterThrottleKillTimer / device.starterThrottleKillTimerStart
			device.starterIgnitionErrorChance = killCoefFac
				* 6
				* linearScale(device.thermals.engineBlockTemperature, -270, 15, 1, 0)
			killCoefFac = math.pow(killCoefFac, 8) * 0.05
		end
		device.starterThrottleKillCoef = device.starterThrottleKillCoefSmoother:get(killCoefFac, dt)

		-- use lower starter max av multiplier in case the engine just doesnt start
		-- occasionally this would result in the engine starting and immediately shutting down, so its disabled
		local starterMaxAVMultiplier = 1.1 -- math.min(1.1, device.outputAV1/device.starterMaxAV+(device.starterThrottleKillTimer == 0 and 0 or math.max(2.0, 1.1)))

		-- Initialize smoothed pitch value if not exists
		device.smoothedPitch = device.smoothedPitch or 0.5

		-- Calculate pitch with a more natural curve at low RPMs
		-- Use a logarithmic curve to make low RPMs sound more natural
		local rpmRatio = math.abs(device.outputAV1) / (device.starterMaxAV * starterMaxAVMultiplier)

		-- Apply a logarithmic curve that's more natural for engine sounds
		-- This will make the pitch increase more slowly at lower RPMs
		local curvedRatio = math.log(1 + rpmRatio * 2) / math.log(3)

		-- Set pitch range for more natural sound
		local minPitch = 0.0 -- Lower minimum pitch for deeper sound at low RPM
		local maxPitch = 0.8 -- Slightly reduced max pitch for more realism

		-- Calculate final pitch with limits and apply a small offset to prevent extreme lows
		local targetPitch = minPitch + (maxPitch - minPitch) * curvedRatio

		-- Apply smoothing to prevent sudden pitch changes
		local smoothingFactor = 0.25
		device.smoothedPitch = device.smoothedPitch or targetPitch
		device.smoothedPitch = device.smoothedPitch + (targetPitch - device.smoothedPitch) * smoothingFactor

		-- Apply the smoothed pitch to the sounds
		obj:setPitch(device.engineMiscSounds.starterSoundEngine, device.smoothedPitch)
		if device.engineMiscSounds.starterSoundExhaust then
			obj:setPitch(device.engineMiscSounds.starterSoundExhaust, device.smoothedPitch)
		end

		if device.outputAV1 > device.idleAV * 1.1 then
			device.starterThrottleKillTimer = 0
			device.starterEngagedCoef = 0
			device.starterThrottleKillCoef = 1
			device.starterDisabled = false
			device.starterThrottleKillCoefSmoother:set(device.starterThrottleKillCoef)
			device.starterIgnitionErrorChance = 0
			obj:stopSFX(device.engineMiscSounds.starterSoundEngine)
			if device.engineMiscSounds.starterSoundExhaust then
				obj:stopSFX(device.engineMiscSounds.starterSoundExhaust)
			end
		end
	end

	-- Get current RPM
	local currentRPM = device.outputAV1 * avToRPM

	-- Glow plug system logic
	local glow = device.glowPlug
	local starterActive = device.starterEngagedCoef > 0
	local engineRunning = currentRPM > (device.idleRPM * 0.5) -- Use a stable RPM threshold for "running"
	local engineBlockTemp = device.thermals and device.thermals.engineBlockTemperature or 20
	local ignitionLevel = electrics.values.ignitionLevel or 0

	if device.isDieselEngine then
		-- State Machine Transitions
		if ignitionLevel <= 0 then
			if glow.state ~= "off" then
				glow.state = "off"
				if glow.debug then
					print("[GlowPlug] System OFF")
				end
			end
		elseif engineRunning then
			if glow.state ~= "postheat" and glow.state ~= "off" then
				glow.state = "postheat"
				glow.preheatTimer = 30 -- Post-heat for 30 seconds
				if glow.debug then
					print("[GlowPlug] Transition to POSTHEAT (30s)")
				end
			end
		elseif starterActive then
			if glow.state ~= "assist" then
				glow.state = "assist"
				if glow.debug then
					print("[GlowPlug] Transition to ASSIST")
				end
			end
		else -- Ignition on, engine not running, not cranking
			if glow.state == "off" or glow.state == "postheat" then
				glow.state = "preheat"
				-- Calculate preheat time: 20C -> 0s, 0C -> 5s, -20C -> 10s
				glow.preheatTimer = clamp((20 - engineBlockTemp) / 4, 0, 15)
				if glow.debug then
					print(
						string.format(
							"[GlowPlug] Transition to PREHEAT (%.1fs at %.1fC)",
							glow.preheatTimer,
							engineBlockTemp
						)
					)
				end
			end
		end

		-- Update timers
		if glow.state == "preheat" or glow.state == "postheat" then
			glow.preheatTimer = max(0, glow.preheatTimer - dt)
			if glow.state == "postheat" and glow.preheatTimer <= 0 then
				glow.state = "off"
				if glow.debug then
					print("[GlowPlug] Post-heat finished, system OFF")
				end
			end
		end

		-- Update heat (exponential-ish heating and decay)
		local isHeating = glow.state ~= "off" and (glow.state ~= "postheat" or glow.preheatTimer > 0)

		if isHeating then
			-- Heat up: 0 to 1 in ~4 seconds
			glow.heat = min(1, glow.heat + dt * 0.25)
		else
			-- Cool down: 1 to 0 in ~8 seconds
			glow.heat = max(0, glow.heat - dt * 0.125)
		end

		-- Export to electrics for UI/Dashboard
		electrics.values.glowPlugsActive = (glow.state ~= "off" and glow.heat > 0.1) and 1 or 0
		electrics.values.waitToStart = (glow.state == "preheat" and glow.preheatTimer > 0) and 1 or 0

		if glow.debug then
			-- Add periodic debug print for state/heat
			device.glowDebugTimer = (device.glowDebugTimer or 0) - dt
			if device.glowDebugTimer <= 0 then
				print(
					string.format(
						"[GlowPlug] State: %s, Heat: %.2f, Timer: %.1f",
						glow.state,
						glow.heat,
						glow.preheatTimer
					)
				)
				device.glowDebugTimer = 1.0
			end
		end
	else
		glow.state = "off"
		glow.heat = 0
	end

	-- Update battery state

	-- Local function to initialize battery parameters
	local function initBattery(device, jbeamData)
		-- Set battery parameters based on system voltage (12V or 24V)
		local is24V = device.batterySystemVoltage == 24

		-- Set voltage thresholds based on system voltage
		device.batteryNominalVoltage = is24V and 27.6 or 13.8 -- 27.6V for 24V, 13.8V for 12V when fully charged
		device.batteryMinVoltage = is24V and 18.0 or 9.0 -- 18V for 24V, 9V for 12V systems
		device.batteryCutoffVoltage = is24V and 16.0 or 8.0 -- Absolute minimum voltage before complete cutoff
		device.batteryWarningVoltage = is24V and 22.0 or 11.0 -- Voltage when warning indicators activate
		device.batteryLowVoltage = is24V and 20.0 or 10.0 -- Voltage when systems start to fail

		-- Set charge and drain rates based on system voltage
		device.batteryChargeRate = is24V and 1.0 or 0.5 -- Higher charge rate for 24V systems
		device.batteryDrainRate = is24V and 30.0 or 15.0 -- Base drain rate when cranking (A)

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
		log(
			"I",
			"combustionEngine.initBattery",
			string.format(
				"Battery initialized: %.1fV system, %.1fAh capacity",
				device.batterySystemVoltage,
				device.batteryCapacity
			)
		)
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

	if not device.batteryOverride then
		if starterActive and not engineRunning then
			-- Drain battery when starting (higher drain for 24V systems)
			local drainRate = (device.batteryDrainRate or 15.0) * (device.batteryDrainScale or 1.0)
			device.batteryCharge =
				math.max(0, device.batteryCharge - (drainRate * dt) / ((device.batteryCapacity or 100.0) * 3600))
			device.batteryLoad = drainRate -- Track current load in Amps
		elseif engineRunning then
			-- Recharge battery when engine is running above idle
			-- Charge rate is higher for 24V systems and scales with RPM
			local chargeRate = (device.batteryChargeRate or 0.5) * (device.outputAV1 / math.max(1, device.idleAV))
			device.batteryCharge = math.min(1.0, device.batteryCharge + (chargeRate * dt) / 3600)
			device.batteryLoad = -chargeRate -- Negative load indicates charging
		end
	end

	-- Add glow plug load to battery
	if glow.state ~= "off" and not device.batteryOverride then
		device.batteryLoad = device.batteryLoad + (glow.maxAmps * glow.heat)
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
	local chargeCurve = math.pow(charge, 0.7) -- More pronounced voltage drop at lower charge
	device.batteryVoltage = minVoltage + (maxVoltage - minVoltage) * chargeCurve

	-- Ensure voltage stays within bounds
	device.batteryVoltage = math.max(cutoffVoltage, math.min(maxVoltage, device.batteryVoltage))

	-- Calculate battery voltage factor for lights (0.5 to 1.0 range)
	-- Lights will start dimming below warning voltage
	local dimStartVoltage = device.batteryWarningVoltage or (is24V and 22.0 or 11.0)
	dimStartVoltage = tonumber(dimStartVoltage) or (is24V and 22.0 or 11.0)

	-- Ensure dimStartVoltage is between cutoff and max voltage
	dimStartVoltage = math.max(cutoffVoltage * 1.1, math.min(maxVoltage * 0.9, dimStartVoltage))

	-- Calculate full brightness voltage (slightly below nominal)
	local fullBrightnessVoltage = maxVoltage * 0.95 -- 95% of nominal

	-- Ensure fullBrightnessVoltage is above dimStartVoltage
	fullBrightnessVoltage = math.max(dimStartVoltage * 1.05, fullBrightnessVoltage)

	-- Calculate brightness factor with safety checks
	local batteryBrightnessFactor = linearScale(
		math.max(cutoffVoltage, math.min(maxVoltage, device.batteryVoltage)),
		dimStartVoltage,
		fullBrightnessVoltage,
		0.5, -- Minimum brightness factor
		1.0 -- Maximum brightness factor
	)

	-- Update electrical system with current battery state
	-- We provide multiple names to ensure compatibility with various dashboard gauges across different vehicles
	if electrics.values then
		electrics.values.batteryVoltage = device.batteryVoltage
		electrics.values.voltage = device.batteryVoltage
		electrics.values.volts = device.batteryVoltage
		electrics.values.Volts = device.batteryVoltage
		electrics.values.batteryCharge = device.batteryCharge
		electrics.values.batteryCurrent = device.batteryLoad or 0
		electrics.values.amps = device.batteryLoad or 0
		electrics.values.Amps = device.batteryLoad or 0
	end
	batteryBrightnessFactor = math.max(0.2, math.min(1.0, batteryBrightnessFactor)) -- Clamp to 20-100%

	-- Base brightness based on RPM - starts dim and increases with RPM
	-- At 0 RPM: 0.4 (dim)
	-- At cranking RPM (200): ~0.5
	-- At idle (800): ~0.76
	-- At max RPM: 0.8
	-- Base brightness based on RPM - starts bright and increases slightly
	local baseBrightness = linearScale(currentRPM, 0, device.maxRPM, 0.8, 1.0) * batteryBrightnessFactor

	-- Starter effect - when cranking, we want to see the brightness dim
	local dimmingEffect = 1.0
	if device.starterEngagedCoef > 0 then
		-- Dim the lights significantly when cranking
		-- Add a subtle flicker effect
		local flicker = (math.random() - 0.5) * 0.1 -- +/- 0.05 variation
		dimmingEffect = 0.6 + flicker
	end

	-- Combine effects
	local brightness = baseBrightness * dimmingEffect

	-- Calculate electrical load coefficient based on battery state and brightness
	-- Lower battery voltage will reduce the electrical load coefficient more
	local loadWarnVoltage = device.batteryWarningVoltage * 0.65 -- Start warning slightly earlier for load reduction
	local loadMinVoltage = device.batteryLowVoltage * 0.45 -- Minimum voltage for load reduction

	-- Scale based on system voltage
	local batteryLoadFactor = linearScale(device.batteryVoltage, loadMinVoltage, loadWarnVoltage, 0.5, 1.0)
	batteryLoadFactor = math.max(0.5, math.min(1.0, batteryLoadFactor)) -- Clamp to 50-100%

	-- Apply battery load factor to brightness and ensure we stay within reasonable bounds
	electrics.values.electricalLoadCoef = math.min(math.max(brightness * batteryLoadFactor, 0.3), 1.0)

	-- Update battery drain scale based on electrical load (higher load = faster drain)
	device.batteryDrainScale = 0.5 + (electrics.values.electricalLoadCoef * 1.5) -- 0.5x to 2.0x drain rate

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
	device.fastIgnitionErrorCoef = fastIgnitionErrorCoef < (device.fastIgnitionErrorChance + lowFuelIgnitionErrorChance)
			and 0
		or 1

	if
		device.shutOffSoundRequested
		and device.outputAV1 < device.idleAV * 0.95
		and device.outputAV1 > device.idleAV * 0.5
	then
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
		device.overRevDamage = min(
			max(device.overRevDamage + (abs(device.outputAV1) - device.maxPhysicalAV) * dt / device.maxOverRevDamage, 0),
			1
		)
		local lockupChance = random(60, 100) * 0.01
		local valveHitChance = random(10, 60) * 0.01
		if
			lockupChance <= device.overRevDamage
			and not damageTracker.getDamage("engine", "catastrophicOverrevDamage")
		then
			device:lockUp()
			damageTracker.setDamage("engine", "catastrophicOverrevDamage", true)
			guihooks.message({
				txt = "vehicle.combustionEngine.engineCatastrophicOverrevDamage",
				context = {},
			}, 4, "vehicle.damage.catastrophicOverrev")

			if #device.engineBlockNodes >= 2 then
				sounds.playSoundOnceFollowNode("event:>Vehicle>Failures>engine_explode", device.engineBlockNodes[1], 1)

				for i = 1, 50 do
					local rnd = random()
					obj:addParticleByNodesRelative(
						device.engineBlockNodes[2],
						device.engineBlockNodes[1],
						i * rnd,
						43,
						0,
						1
					)
					obj:addParticleByNodesRelative(
						device.engineBlockNodes[2],
						device.engineBlockNodes[1],
						i * rnd,
						39,
						0,
						1
					)
					obj:addParticleByNodesRelative(
						device.engineBlockNodes[2],
						device.engineBlockNodes[1],
						-i * rnd,
						43,
						0,
						1
					)
					obj:addParticleByNodesRelative(
						device.engineBlockNodes[2],
						device.engineBlockNodes[1],
						-i * rnd,
						39,
						0,
						1
					)
				end
			end
		end
		if valveHitChance <= device.overRevDamage then
			device:scaleOutputTorque(0.98, 0.2)
			damageTracker.setDamage("engine", "mildOverrevDamage", true)
			guihooks.message({
				txt = "vehicle.combustionEngine.engineMildOverrevDamage",
				context = {},
			}, 4, "vehicle.damage.mildOverrev")
		end
	end

	if device.maxTorqueRating > 0 then
		damageTracker.setDamage("engine", "overTorqueDanger", device.combustionTorque > device.maxTorqueRating)
		if device.combustionTorque > device.maxTorqueRating then
			local torqueDifference = device.combustionTorque - device.maxTorqueRating
			device.overTorqueDamage = min(device.overTorqueDamage + torqueDifference * dt, device.maxOverTorqueDamage)
			if
				device.overTorqueDamage >= device.maxOverTorqueDamage
				and not damageTracker.getDamage("engine", "catastrophicOverTorqueDamage")
			then
				device:lockUp()
				damageTracker.setDamage("engine", "catastrophicOverTorqueDamage", true)
				guihooks.message({
					txt = "vehicle.combustionEngine.engineCatastrophicOverTorqueDamage",
					context = {},
				}, 4, "vehicle.damage.catastrophicOverTorque")

				if #device.engineBlockNodes >= 2 then
					sounds.playSoundOnceFollowNode(
						"event:>Vehicle>Failures>engine_explode",
						device.engineBlockNodes[1],
						1
					)

					for i = 1, 3 do
						local rnd = random()
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							i * rnd * 3,
							43,
							0,
							9
						)
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							i * rnd * 3,
							39,
							0,
							9
						)
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							-i * rnd * 3,
							43,
							0,
							9
						)
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							-i * rnd * 3,
							39,
							0,
							9
						)

						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							i * rnd * 3,
							56,
							0,
							1
						)
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							i * rnd * 3,
							57,
							0,
							1
						)
						obj:addParticleByNodesRelative(
							device.engineBlockNodes[2],
							device.engineBlockNodes[1],
							i * rnd * 3,
							58,
							0,
							1
						)
					end
				end
			end
		end
	end

	-- calculate the actual current idle torque to check for lockup conditions due to high friction
	local idleThrottle = device.maxIdleThrottle
	local idleTorque = (device.torqueCurve[floor(abs(device.idleAV) * avToRPM)] or 0) * device.intakeAirDensityCoef
	local idleThrottleMap = min(
		max(
			idleThrottle
				+ idleThrottle
					* device.maxPowerThrottleMap
					/ (idleTorque * device.forcedInductionCoef * abs(device.outputAV1) + 1e-30)
					* (1 - idleThrottle),
			0
		),
		1
	)
	idleTorque = ((idleTorque * device.forcedInductionCoef * idleThrottleMap) + device.nitrousOxideTorque)

	local finalFriction = device.friction * device.wearFrictionCoef * device.damageFrictionCoef
	local finalDynamicFriction = device.dynamicFriction
		* device.wearDynamicFrictionCoef
		* device.damageDynamicFrictionCoef
	local frictionTorque = finalFriction - (finalDynamicFriction * device.idleAV)

	if
		not device.isDisabled
		and (
			frictionTorque > device.maxTorque
			or (device.outputAV1 < device.idleAV * 1.1 and frictionTorque > idleTorque * 0.95)
		)
	then
		-- if our friction is higher than the biggest torque we can output, the engine WILL lock up automatically
		-- however, we need to communicate that with other subsystems to prevent issues, so in this case we ADDITIONALLY lock it up manually
		-- device:lockUp()
	end

	local compressionBrakeCoefAdjusted = device.throttle > 0 and 0 or device.compressionBrakeCoefDesired
	if compressionBrakeCoefAdjusted ~= device.compressionBrakeCoefActual then
		device.compressionBrakeCoefActual = compressionBrakeCoefAdjusted
		device:setEngineSoundParameter(
			device.engineSoundIDExhaust,
			"compression_brake_coef",
			device.compressionBrakeCoefActual,
			"exhaust"
		)
	end

	local antiLagCoefAdjusted = device.antiLagCoefDesired
	if antiLagCoefAdjusted ~= device.antiLagCoefActual then
		device.antiLagCoefActual = antiLagCoefAdjusted
		device:setEngineSoundParameter(
			device.engineSoundIDExhaust,
			"triggerAntilag",
			device.antiLagCoefActual,
			"exhaust"
		)
		device.turbocharger.setAntilagCoef(device.antiLagCoefActual)
	end

	device.exhaustFlowDelay:push(device.engineLoad)

	-- push our summed fuels into the delay lines (shift fuel does not have any delay and therefore does not need a line)
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

	device.intakeAirDensityCoef = obj:getRelativeAirDensity() * device.airRestrictionMultiplier

	device:checkHydroLocking(dt)

	device.idleAVReadError = device.idleAVReadErrorSmoother:getUncapped(
		device.idleAVReadErrorRangeHalf - random(device.idleAVReadErrorRange),
		dt
	) * device.wearIdleAVReadErrorRangeCoef * device.damageIdleAVReadErrorRangeCoef
	device.idleAVStartOffset =
		device.idleAVStartOffsetSmoother:get(device.idleAV * device.idleStartCoef * device.starterEngagedCoef, dt)
	device.maxIdleAV = device.idleAV
		+ device.idleAVReadErrorRangeHalf
			* device.wearIdleAVReadErrorRangeCoef
			* device.damageIdleAVReadErrorRangeCoef
	device.minIdleAV = device.idleAV
		- device.idleAVReadErrorRangeHalf
			* device.wearIdleAVReadErrorRangeCoef
			* device.damageIdleAVReadErrorRangeCoef

	device.gfxTimer = (device.gfxTimer or 0) + dt
	if device.gfxTimer > 0.1 then -- 10Hz update to UI
		local cylStates = {}
		if device.cylinders then
			for i = 1, #device.cylinders do
				cylStates[i] = device.cylinders[i].failMode
			end
		end
		guihooks.trigger("EngineFailureStates", {
			name = device.name or "mainEngine",
			cylinders = cylStates,
			fuelPressure = device.fuelPressureMultiplier or 1.0,
			airRestriction = device.airRestrictionMultiplier or 1.0,
		})
		-- Log once per 5 seconds to avoid spamming while verifying connection
		device.debugPulse = (device.debugPulse or 0) + dt
		if device.debugPulse > 5.0 then
			log("I", "combustionEngine.debug", "UI Sync active for device: " .. (device.name or "mainEngine"))
			device.debugPulse = 0
		end
		device.gfxTimer = 0
	end

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
	local correctedThrottle = -throttle
			* min(max(engineAV - limiterAV, 0), device.revLimiterMaxAVOvershoot)
			* device.invRevLimiterRange
		+ throttle

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
		-- Deactivate the limiter once below the deactivation threshold
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
		-- Deactivate the limiter once below the deactivation threshold
		local revLimiterAVThreshold = min(limiterAV - device.revLimiterAVDrop, limiterAV)
		device.revLimiterActive = engineAV > revLimiterAVThreshold
		device.revLimiterWasActiveTimer = 0
		return 0
	end

	return throttle
end

local function updateFixedStep(device, dt)
	-- update idle throttle
	device.idleTimer = device.idleTimer - dt
	if device.idleTimer <= 0 then
		local idleTimeRandomCoef = linearScale(device.idleTimeRandomness, 0, 1, 1, randomGauss3() * 0.6666667)
		device.idleTimer = device.idleTimer + device.idleTime * idleTimeRandomCoef
		-- device.idleTime
		local engineAV = device.outputAV1
		local highIdle = device.idleAV
			+ math.max(math.min(60 + linearScale(device.thermals.engineBlockTemperature, 60, -60, -60, 60), 250), 0)
				* 0.6 -- ((max(-device.thermals.engineBlockTemperature, 10)-10) * 0.7)
		local idleAV = max(highIdle, device.idleAVOverwrite)
		local maxIdleThrottle = min(max(device.maxIdleThrottle, device.maxIdleThrottleOverwrite), 1)
		local idleAVError = max(idleAV - engineAV + device.idleAVReadError + device.idleAVStartOffset, 0)
		device.idleThrottleTarget = min(idleAVError * device.idleControllerP, maxIdleThrottle)

		-- print(device.idleThrottle)
	end
	device.idleThrottle = device.idleThrottleSmoother:get(device.idleThrottleTarget, dt)

	device.forcedInductionCoef = 1
	device.turbocharger.updateFixedStep(dt)
	device.supercharger.updateFixedStep(dt)
end

-- velocity update is always nopped for engines

local function updateTorque(device, dt)
	local recoveryFloodThreshold = 0.2
	local floodStartThreshold = 0.1
	local maxFloodThreshold = 1
	local minNormalFlood = 0.3

	local isStarting = (device.startingHesitationPhase < 3)
	local isRunning = (math.abs(device.outputAV1) > 600 * (math.pi / 30)) -- ~600 RPM threshold for running
	local isCranking = device.starterEngagedCoef > 0
		and math.abs(device.outputAV1) < 600 * (math.pi / 30)
		and not isRunning -- Prevent cranking state when engine is running

	local isFlooded = device.floodLevel > (maxFloodThreshold or 1) -- Adjust threshold as needed
	local engineAV = device.outputAV1

	local throttle = (electrics.values[device.electricsThrottleName] or 0)
		* (electrics.values[device.electricsThrottleFactorName] or device.throttleFactor)

	local rawEngineTempC = (device.thermals and device.thermals.engineBlockTemperature) or 20
	-- Clamp temperature between -30 and 120 for physics calculations to prevent 'liquid nitrogen' bug
	local engineTempC = math.max(-30, math.min(120, rawEngineTempC))
	-- Normalize temperature for 0-1 factor physics (0 at -20C, 1 at 80C)
	local engineTempNorm = math.max(0, math.min(1, (engineTempC + 20) / 100))

	-- don't include idle throttle as otherwise idle affects the turbo wastegate, do include it though if we have a raised idle throttle (eg semi truck hidh idle)
	device.requestedThrottle = max(throttle, device.idleAVOverwrite > 0 and device.idleThrottle or 0)

	throttle = min(
		max(
			max(device.idleThrottle, throttle)
				* (device.starterThrottleKillCoef + (1 - device.starterIgnitionErrorCoef) * device.inertia * 0.1)
				* device.ignitionCoef,
			0
		),
		1
	)

	throttle = device:applyRevLimiter(engineAV, throttle, dt)

	-- smooth our actual throttle value to simulate various effects in a real engine that do not allow immediate throttle changes
	throttle = device.throttleSmoother:getUncapped(throttle, dt) -- * 1.2
	local finalFriction = device.friction * device.wearFrictionCoef * device.damageFrictionCoef
	local finalDynamicFriction = device.dynamicFriction
		* device.wearDynamicFrictionCoef
		* device.damageDynamicFrictionCoef

	local tableRPM = floor(engineAV * avToRPM) or 0
	local torque = (device.torqueCurve[tableRPM] or 0) * device.intakeAirDensityCoef
	local maxCurrentTorque = torque - finalFriction - (finalDynamicFriction * engineAV)
	-- blend pure throttle with the constant power map
	local throttleMap = smoothmin(
		max(
			throttle
				+ throttle
					* device.maxPowerThrottleMap
					/ (torque * device.forcedInductionCoef * engineAV + 1e-30)
					* (1 - throttle),
			0
		),
		1,
		(1 - throttle) * 0.8
	) -- 0.8 can be tweaked to adjust the peakiness of the throttlemap adjusted torque curve

	local ignitionCut = device.ignitionCutTime > 0
	torque = (torque * device.forcedInductionCoef * throttleMap) + device.nitrousOxideTorque

	-- Apply fuel properties
	local fuelProps = device.currentFuelProperties or fuelProperties.gasoline
	torque = torque * (fuelProps.energyDensity or 1.0)

	-- Accumulate fuel damage if incompatible
	if device.fuelIncompatible then
		local damageRate = 0.05 -- Damage per second of operation
		device.fuelDamage = min(1, device.fuelDamage + damageRate * dt)

		-- Apply torque reduction based on fuel damage
		torque = torque * (1 - device.fuelDamage * 0.7) -- Up to 70% power loss

		-- Increase misfire chance based on octane rating mismatch
		local requiredOctane = fuelProperties[device.requiredEnergyType]
				and fuelProperties[device.requiredEnergyType].octaneRating
			or 91
		if fuelProps.octaneRating < requiredOctane then
			local octaneMismatch = requiredOctane - fuelProps.octaneRating
			device.fastIgnitionErrorChance = min(1, device.fastIgnitionErrorChance + octaneMismatch * 0.01 * dt)
		end
	end

	torque = torque
		* device.outputTorqueState
		* (ignitionCut and 0 or 1)
		* device.slowIgnitionErrorCoef
		* device.fastIgnitionErrorCoef
		* device.starterIgnitionErrorCoef
	-- torque = min(torque, device.maxTorqueLimit)  --limit output torque to a specified max, math.huge by default

	local lastInstantEngineLoad = device.instantEngineLoad
	local instantLoad =
		min(max(torque / ((maxCurrentTorque + 1e-30) * device.outputTorqueState * device.forcedInductionCoef), 0), 1)
	device.instantEngineLoad = instantLoad
	device.engineLoad = device.loadSmoother:getCapped(device.instantEngineLoad, dt)
	local normalizedEngineAV = clamp(engineAV / device.maxAV, 0, 1)
	local revLimiterActive = device.revLimiterWasActiveTimer < 0.1
	device.exhaustFlowCoef = revLimiterActive and (device.revLimiterActiveMaxExhaustFlowCoef * normalizedEngineAV)
		or device.engineLoad

	local absEngineAV = abs(engineAV)
	local dtT = dt * torque
	local dtTNitrousOxide = dt * device.nitrousOxideTorque

	local burnEnergy = dtT * (dtT * device.halfInvEngInertia + engineAV)
	local burnEnergyNitrousOxide = dtTNitrousOxide * (dtTNitrousOxide * device.halfInvEngInertia + engineAV)
	device.engineWorkPerUpdate = device.engineWorkPerUpdate + burnEnergy
	device.frictionLossPerUpdate = device.frictionLossPerUpdate + finalFriction * absEngineAV * dt
	device.pumpingLossPerUpdate = device.pumpingLossPerUpdate + finalDynamicFriction * engineAV * engineAV * dt
	local invBurnEfficiency = device.invBurnEfficiencyTable[floor(device.instantEngineLoad * 100)]
		* device.invBurnEfficiencyCoef
	device.spentEnergy = device.spentEnergy + burnEnergy * invBurnEfficiency
	device.spentEnergyNitrousOxide = device.spentEnergyNitrousOxide + burnEnergyNitrousOxide * invBurnEfficiency

	local compressionBrakeTorque = (device.compressionBrakeCurve[tableRPM] or 0) * device.compressionBrakeCoefActual
	-- todo check why this is not included in thermals
	local engineBrakeTorque = device.engineBrakeTorque * (1 - min(instantLoad + device.antiLagCoefActual, 1))
	local frictionTorque = finalFriction + finalDynamicFriction * absEngineAV + engineBrakeTorque
	-- friction torque is limited for stability
	frictionTorque = min(frictionTorque, absEngineAV * device.inertia * 2000) * sign(engineAV)

	-- Initialize flood level tracking if needed
	device.floodLevel = device.floodLevel or 0

	-- Track starter engagement state
	local starterEngagedThisFrame = device.starterEngagedCoef > 0 and (device.lastStarterEngagedCoef or 0) <= 0
	local starterDisengagedThisFrame = device.starterEngagedCoef <= 0 and (device.lastStarterEngagedCoef or 0) > 0

	-- Initialize or update starter engagement timer
	if starterEngagedThisFrame then
		device.starterEngageTimer = 0.01 -- Duration of initial engagement effect
	elseif starterDisengagedThisFrame then
		device.starterEngageTimer = 0
	end

	-- Calculate engagement factor
	local engagementFactor = 1.0
	if device.starterEngageTimer and device.starterEngageTimer > 0 then
		local engagementCurve = smoothstep(0, 0.7, 0.7 - device.starterEngageTimer)
		engagementFactor = 0.8 + 1.2 * engagementCurve -- Start at 80% torque and ramp up
		device.starterEngageTimer = device.starterEngageTimer - dt
	end

	local isDiesel = (device.requiredEnergyType == "diesel")
		or (device.engineType and (device.engineType == "diesel" or device.engineType == "dieselElectric"))
	local batteryVoltageFactor = electrics.values.batteryVoltageFactor or 1.0

	-- Calculate peak starter torque using original multipliers
	local maxTorqueAtZeroRPM = isDiesel and (device.starterTorque or device.starterMaxAV * 18.95)
		or (device.starterTorque or device.starterMaxAV * 17.66)
	if device.starterTorqueOverride and device.starterTorqueOverride > 0 then
		maxTorqueAtZeroRPM = device.starterTorqueOverride
	end

	-- High-fidelity effects
	local voltageEffect = math.pow(batteryVoltageFactor, 0.8)
	local wearFactor = 1.0 - ((device.starterWear or 0) * 0.4)
	-- Clamp flood reduction to prevent negative torque/reverse cranking
	local floodTorqueReduction = math.min(0.75, (device.floodLevel * 1.5) * 0.5)

	-- Temperature effect - maintaining improved sensitivity
	local normalizedTemp = math.max(0, math.min(1, (engineTempC + 20) / 100))
	local tempEffect = isDiesel and (0.6 + normalizedTemp * 0.6) or (0.7 + normalizedTemp * 0.5)

	local finalMaxTorque = maxTorqueAtZeroRPM
		* voltageEffect
		* wearFactor
		* (1 - floodTorqueReduction)
		* tempEffect
		* engagementFactor

	-- Speed-based torque reduction
	local engineSpeedFactor = math.max(0, 1 - math.abs(engineAV) * device.invStarterMaxAV)
	local baseStarterTorque = device.starterEngagedCoef	* finalMaxTorque * engineSpeedFactor * (device.startingHesitationFactor or 1.0)

	-- Add extra resistance during initial engagement and when cold
	if device.starterEngageTimer and device.starterEngageTimer > 0.25 then
		local engagementResistance = (1 - (device.starterEngageTimer - 0.25) * 4) * 0.4
		local coldResistance = math.max(0, (0 - engineTempC) / 15)
		frictionTorque = frictionTorque * (1 + engagementResistance * (1 + coldResistance))
	end

	-- Calculate final starter torque with velocity-based reduction and engagement
	-- Multiplies starter engagement (0-1) by base torque, then scales based on engine speed

	-- Initialize compression and cylinder states if not exists
	device.compressionOscTimer = device.compressionOscTimer or 0
	device.compressionState = device.compressionState or 0
	device.compressionStateTimer = device.compressionStateTimer or 0

	-- Initialize per-cylinder fuel and combustion state
	if not device.cylinders then
		device.cylinders = {}
		local cylinderCount = device.fundamentalFrequencyCylinderCount or 4
		for i = 1, cylinderCount do
			device.cylinders[i] = {
				fuelAmount = 0, -- Amount of fuel in cylinder (0-1)
				airAmount = device.intakeAirDensityCoef or 0.8, -- Amount of air in cylinder (0-1)
				compressionRatio = 8, -- Engine compression ratio
				isCompressing = false, -- Whether cylinder is in compression stroke
				isFiring = false, -- Whether cylinder is in power stroke
				sparkPlugFouled = false, -- Whether spark plug is fouled (gasoline only)
				lastFired = -1, -- Last cycle this cylinder fired
				misfireCount = 0, -- Consecutive misfires
				temperature = 0, -- Current temperature (for heat simulation)
				damage = 0, -- Cylinder damage (0-1)
				lastFuelAddTime = -1, -- When was fuel last added (to prevent rapid adding)
				failMode = "none", -- Failure mode: none, dead, leak, broken
			}
		end
	end

	-- Initialize flood level and choke effect if not set
	device.floodLevel = device.floodLevel or 0
	device.chokeEffect = device.chokeEffect or 0 -- Initialize choke effect (0 = no choke, 1 = full choke)

	-- Get engine temperature in Celsius (convert from Kelvin)
	-- Flood recovery and prevention with more forgiving values
	-- Base rates (slower increase, faster recovery)
	local floodRecoveryRate = 0.01 -- Increased base recovery rate (was 5)
	local floodIncreaseRate = 0.2 -- Significantly reduced base increase rate (was -0.2)

	-- Temperature effect - more recovery when warm, less flooding when cold
	local tempFactor = clamp((engineTempC + 30) / 80, 0.2, 1.8) -- Wider range, less extreme at cold

	-- Adjust rates based on conditions
	if isCranking then
		-- When cranking, limit how quickly flooding can increase based on temperature
		local coldEffect = math.max(0.2, 1.0 - ((engineTempC + 30) / 120)) -- More gradual cold effect
		floodIncreaseRate = floodIncreaseRate * (0.1 + coldEffect * 0.6) * 2.0 -- Increased flooding chance when cranking

		-- Increase recovery rate more when cranking to help clear flooding
		floodRecoveryRate = floodRecoveryRate * 0.8 * tempFactor -- Reduced recovery during normal cranking
	else
		-- When engine is running, recover faster
		floodRecoveryRate = floodRecoveryRate * 1.5 -- Reduced from 2x to 1.5x
	end

	-- Clear flood mode - hold throttle to clear flooded engine
	if isCranking and throttle > 0.7 and device.outputAV1 < 1000 * (math.pi / 30) then -- Slightly more lenient RPM threshold
		-- More aggressive clearing when throttle is held open
		local clearFactor = 8.0 + (throttle * 0.7) -- 8x at 0.7 throttle, up to 14x at WOT (reduced from 12x-20x)
		floodRecoveryRate = floodRecoveryRate * clearFactor

		-- Reduce choke effect when clearing flood, but not as aggressively
		device.chokeEffect = device.chokeEffect * 0.7 -- Was 0.5

		-- If RPM is very low, help clear faster (cranking or just after start)
		if math.abs(device.outputAV1) < 100 * (math.pi / 30) then
			floodRecoveryRate = floodRecoveryRate * 1.5
		end
	end

	-- Calculate flood changes based on engine state
	local floodChangeRate = 0
	if isCranking and math.random() < 0.09 then -- Reduced chance of increasing flood
		floodChangeRate = floodIncreaseRate * dt -- Scale by delta time
	else
		-- Recover faster when engine is running well or throttle is open (clear flood mode)
		local recoveryMultiplier = isRunning and 3.0 or 1.0
		if throttle > 0.8 then
			recoveryMultiplier = recoveryMultiplier * 2
		end -- Clear flood mode
		floodChangeRate = -floodRecoveryRate * dt * 10 * recoveryMultiplier
	end

	-- Apply flood changes with some randomness and smoothing
	local randomFactor = 0.95 + math.random() * 0.1 -- 0.95 to 1.05 (tighter random range)
	local newFloodLevel = device.floodLevel + (floodChangeRate * randomFactor)

	-- Apply limits with hysteresis to prevent rapid bouncing
	if floodChangeRate > 0 then -- When increasing flood
		-- Slower increase when already flooded
		local increaseDamping = 1.0 - (device.floodLevel * 0.5) -- 100% at 0%, 50% at 100% flood
		newFloodLevel = device.floodLevel + ((newFloodLevel - device.floodLevel) * increaseDamping)
		newFloodLevel = math.min(0.85, newFloodLevel) -- Cap at 85% to prevent max flooding
	else -- When recovering
		-- Faster recovery when more flooded
		local recoveryBoost = 0.1 + (device.floodLevel * 0.2) -- 1x at 0%, 3x at 100% flood
		newFloodLevel = device.floodLevel + ((newFloodLevel - device.floodLevel) * recoveryBoost)
		newFloodLevel = math.max(0, math.min(1.0, newFloodLevel))
	end

	-- Update the global flood level
	device.floodLevel = newFloodLevel

	-- Debug settings with rate limiting and more detailed output
	local debugFuel = false
	device.lastFloodLogTime = device.lastFloodLogTime or 0
	local currentTime = os.clock()
if debugFuel and (currentTime - device.lastFloodLogTime) > 2.0 then
    -- Only log if something interesting is happening
    if device.floodLevel > 0.05 and isCranking then
        -- Calculate current torque reduction for debugging
        local floodTorqueReduction = math.min(0.75, (device.floodLevel * 1.5) * 0.5)
        local remainingTorque = (1 - floodTorqueReduction) * 100
        
        -- Log comprehensive flood info
        log(
            "I",
            "Flooding",
            string.format(
                "Flood: %.1f%% | Torque Loss: %.1f%% | Remaining: %.1f%% | RPM: %.1f | Cranking: %s",
                device.floodLevel * 100,
                floodTorqueReduction * 100,
                remainingTorque,
                math.abs(device.outputAV1) * 9.5493,
                tostring(isCranking)
            )
        )
        
        device.lastFloodLogTime = currentTime
    end
end
	-- Engine state flags - more accurate state detection

	-- Temperature handling - engine temperatures are in Celsius
	local engineTempC = (device.thermals and device.thermals.engineBlockTemperature) or 20
	local ambientTempC = (device.oil or 15) -- Default to 15°C

	-- Temperature effect on starter torque (reduces torque in cold conditions)
	local tempEffectOnStarter = 1.0 - math.max(0, math.min(0.85, (0 - engineTempC) / 25)) -- Steeper drop off, max 85% reduction

	-- Cold start enrichment using temperature-based lookup table (reduced values)
	local function getColdEnrichment(tempC)
		-- Temperature in Celsius to enrichment factor mapping
		-- [tempC] = enrichmentMultiplier
		local enrichmentMap = {
			[-30] = 3.0, -- Reduced from 4.0
			[-20] = 2.6, -- Reduced from 3.5
			[-10] = 2.2, -- Reduced from 3.0
			[0] = 1.8, -- Reduced from 2.5
			[10] = 1.5, -- Reduced from 2.0
			[20] = 1.3, -- Reduced from 1.5
			[30] = 1.15, -- Reduced from 1.25
			[40] = 1.05, -- Reduced from 1.1
			[50] = 1.02, -- Reduced from 1.05
			[60] = 1.0,
			[70] = 1.0,
		}

		-- Find the two closest temperature points
		local lowerTemp = -40
		local upperTemp = 80
		local lowerEnrich = 3.0
		local upperEnrich = 0.85

		-- Find the two closest temperature points in the map
		for temp, _ in pairs(enrichmentMap) do
			if temp <= tempC and temp > lowerTemp then
				lowerTemp = temp
				lowerEnrich = enrichmentMap[temp]
			end
			if temp >= tempC and temp < upperTemp then
				upperTemp = temp
				upperEnrich = enrichmentMap[temp]
			end
		end

		-- Linear interpolation between the two closest points
		if lowerTemp == upperTemp then
			return lowerEnrich
		end

		local t = (tempC - lowerTemp) / (upperTemp - lowerTemp)
		return lowerEnrich + (upperEnrich - lowerEnrich) * t
	end

	-- Calculate cold start enrichment based on engine temperature
	local coldStartEnrichment = getColdEnrichment(engineTempC)

	-- Choke effect - increases idle speed and enriches mixture when cold
	local chokeEffect = 0
	local chokeThrottleBoost = 0

	-- Check if we should automatically engage choke based on temperature
	local shouldAutoChoke = (engineTempC < 40) and (isCranking or isStarting)

	-- If carburetor has choke control, use it
	if device.carburetor and device.carburetor.getChokeState then
		-- Let carburetor handle choke logic if available
		local chokeState = device.carburetor:getChokeState(engineTempC, isCranking, isStarting)
		device.isChoked = chokeState.isActive
		chokeEffect = chokeState.effect or 0
		chokeThrottleBoost = chokeState.throttleBoost or 0
	else
		-- Fallback choke logic
		if shouldAutoChoke or (device.isChoked and engineTempC < 60) then
			chokeEffect = math.min(1.0, math.max(0, (60 - engineTempC) / 40))
			chokeThrottleBoost = chokeEffect * 0.02 -- Slight throttle bump when choked
			device.isChoked = true
		else
			device.isChoked = false
		end
	end

	-- Get all fuel and air values from carburetor if available
	local fuelValues = {
		baseFuelAmount = device.carburetor and device.carburetor.baseFuelAmount or 8.0,
		maxFuelPerCylinder = device.carburetor and device.carburetor.maxFuelPerCylinder or 1.0,
		minFuelForInjection = device.carburetor and device.carburetor.minFuelForInjection or 2.0,
		minFuelForCombustion = device.carburetor and device.carburetor.minFuelForCombustion or 0.15,
		minAirForCombustion = device.carburetor and device.carburetor.minAirForCombustion or 0.4,
		fuelEnrichment = device.carburetor and device.carburetor.fuelEnrichment or 1.0,
	}

	-- Get values from carburetor if available
	if device.carburetor and device.carburetor.getFuelValues then
		local carbValues = device.carburetor:getFuelValues(engineTempC, isCranking)
		fuelValues.baseFuelAmount = carbValues.baseFuelAmount
		fuelValues.maxFuelPerCylinder = carbValues.maxFuelPerCylinder
		fuelValues.minFuelForInjection = carbValues.minFuelForInjection
		fuelValues.minFuelForCombustion = carbValues.minFuelForCombustion
		fuelValues.minAirForCombustion = carbValues.minAirForCombustion
	end

	-- Get fuel enrichment from carburetor if available
	if device.carburetor and device.carburetor.getFuelEnrichment then
		fuelValues.fuelEnrichment = device.carburetor:getFuelEnrichment(engineTempC, isCranking, throttle)
	end

	-- Apply fuel enrichment to base fuel amount
	local baseFuelAmount = fuelValues.baseFuelAmount * fuelValues.fuelEnrichment

	-- Local references for cleaner code
	local minFuelForCombustion = fuelValues.minFuelForCombustion
	local minAirForCombustion = fuelValues.minAirForCombustion
	local maxFuelPerCylinder = fuelValues.maxFuelPerCylinder
	local minFuelForInjection = fuelValues.minFuelForInjection
	local fuelEnrichment = fuelValues.fuelEnrichment
	-- Ignition assistance during cranking - more help when cold
	local minIgnitionForCombustion = isCranking and (0.15 * (1.5 - (engineTempC / 100 * 0.8))) or 0.5

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
	local oscFactor2 = math.sin(device.compressionOscTimer * 1.1) * 0.4 -- Slightly detuned for beating effect
	local oscFactor3 = math.sin(device.compressionOscTimer * 0.5) * 0.25 -- Sub-harmonic for low-end rumble

	-- Blend harmonics with emphasis on primary oscillation
	local combinedOscFactor = (oscFactorSharp * 0.7) + (oscFactor2 * 0.2) + (oscFactor3 * 0.1)

	-- Add compression pulses that align with cylinder firing order
	local compressionPulse = 0
	local cylinderCount = device.fundamentalFrequencyCylinderCount or 4 -- Default to 4 cylinders if not set
	local pulsePhase = (device.compressionOscTimer % (math.pi * 2 / cylinderCount)) / (math.pi * 2 / cylinderCount)

	-- Create more pronounced compression pulses with realistic timing
	if pulsePhase > 0.9 and pulsePhase < 1.1 then
		local pulseStrength = 0.8 + math.random() * 0.4 -- Random variation in pulse strength
		-- Shape the pulse with a smooth curve
		local pulseShape = math.sin((pulsePhase - 0.9) * (math.pi / 0.2) * 0.5)
		compressionPulse = pulseStrength * pulseShape * pulseShape
	end

	-- Combine base oscillation with compression pulses
	-- Apply stronger temperature effect - VISUAL shake matches internal resistance
	local normalizedTemp = math.max(0, math.min(1, (engineTempC + 20) / 100)) -- Normalize -20°C to 80°C to 0-1 range
	-- More aggressive temperature curve: 0.2-1.2 multiplier
	local tempEffect = 1.2 - (1 - normalizedTemp) * 1.0

	-- Invert for VISUAL resistance (more shake when cold)
	local resistanceFactor = (2.0 - tempEffect)
	combinedOscFactor = combinedOscFactor * 1.8 * resistanceFactor + compressionPulse * 0.8

	-- Apply a soft clip to prevent extreme values while maintaining peak shape
	combinedOscFactor = math.atan(combinedOscFactor * 0.5) * 1.5

	-- Get battery parameters
	local is24V = device.batterySystemVoltage == 24
	local minVoltage = is24V and 18.0 or 9.0 -- Minimum operating voltage under load
	local maxVoltage = is24V and 28.8 or 14.4 -- Maximum charging voltage
	local nominalVoltage = is24V and 24.0 or 12.0 -- Nominal system voltage

	-- Get current battery state (0.0 to 1.0)
	local chargeLevel = device.batteryCharge or 1.0

	-- Calculate open-circuit voltage (no load)
	local ocv = minVoltage + (maxVoltage - minVoltage) * math.pow(chargeLevel, 1.5)

	-- Battery charge/discharge logic
	local starterCurrent = 0
	local voltageDrop = 0
	local isEngineRunning = device.outputAV1 > device.idleAV * 1.5 -- More robust threshold than starterMaxAV*1.1

	-- Update battery charge based on current conditions
	if device.starterEngagedCoef > 0 then
		-- Base current draw (higher for 24V systems)
		local baseCurrent = is24V and 280 or 140 -- Amps

		-- Current increases with load (lower RPM = higher load)
		local loadFactor = 1.0 - math.min(1.0, math.abs(device.outputAV1) / (device.starterMaxAV * 0.7))
		starterCurrent = baseCurrent * (0.1 + 0.5 * loadFactor) * device.starterEngagedCoef

		-- Internal resistance (higher when battery is cold or discharged)
		local internalResistance = (is24V and 0.02 or 0.04) * (1.0 + (1.0 - chargeLevel) * 2.0)
		voltageDrop = starterCurrent * internalResistance
		-- Calculate energy consumed by starter (in watt-seconds)
		local starterVoltage = ocv - voltageDrop
		local starterPower = starterCurrent * starterVoltage -- Watts
		local energyConsumed = starterPower * dt -- Watt-seconds

		-- Convert energy to battery charge (assuming 50Ah battery capacity)
		local batteryCapacity = 500 * 3600 -- 50Ah in watt-seconds (50A * 12V * 3600s)
		local chargeConsumed = energyConsumed / (batteryCapacity * (is24V and 2 or 1))

		-- Update battery charge
		if device.starterEngagedCoef > 0 and not device.batteryOverride then
			device.batteryCharge = math.max(0, device.batteryCharge - chargeConsumed)
		end
	elseif isEngineRunning and device.starterEngagedCoef == 0 and not device.batteryOverride then
		-- Charge battery when engine is running and starter is off
		local chargeRate = dt * 0.001 -- Base charge rate per second
		device.batteryCharge = math.min(1, device.batteryCharge + chargeRate)
	end

	-- Continuous load drain
	if device.batteryLoad and device.batteryLoad ~= 0 then
		-- Capacity in Amp-seconds. Standard is 50Ah (180,000 As).
		-- We'll use a 10x drain scale so it's clearly visible in the debug UI.
		local capacityAs = 18000 -- 5Ah equivalent for faster debug drain
		local chargeDrained = (device.batteryLoad * dt) / capacityAs
		device.batteryCharge = math.max(0, device.batteryCharge - chargeDrained)
	end

	-- Calculate actual battery voltage under load
	local batteryVoltage = math.max(minVoltage * 0.8, ocv - voltageDrop)

	-- Store values for other systems
	device.batteryVoltage = batteryVoltage
	device.starterCurrent = starterCurrent
	device.alternatorVoltage = isEngineRunning and maxVoltage or 0

	-- Calculate voltage factor (0.0 to 1.0) for torque calculation
	local batteryVoltageFactor = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage)
	batteryVoltageFactor = math.max(0, math.min(1, batteryVoltageFactor)) -- Clamp to 0-1 range

	-- Apply non-linear response curve (more sensitive at lower voltages)
	batteryVoltageFactor = math.pow(batteryVoltageFactor, is24V and 1.5 or 0.6)

	-- Set minimum voltage factor to prevent complete loss of starter torque
	local minVoltageFactor = is24V and 0.15 or 0.1
	batteryVoltageFactor = math.max(minVoltageFactor, batteryVoltageFactor)

	-- Store the calculated battery voltage for other systems
	device.batteryVoltage = batteryVoltage

	-- GaugesDaddy-style voltage simulation
	local gaugeVoltage
	if device.starterEngagedCoef > 0 then
		-- When cranking, show actual battery voltage under load
		gaugeVoltage = batteryVoltage * (is24V and 3.33 or 6.66) -- Scale to match gauge range
	elseif isEngineRunning then
		-- Engine running - show charging voltage (slightly above nominal)
		gaugeVoltage = is24V and 85 or 90 -- 24V system shows ~27V, 12V shows ~14V
	elseif (device.ignitionCoef or 0) > 0.5 then
		-- Ignition on but engine not running - show slightly lower voltage
		gaugeVoltage = is24V and 80 or 85
	else
		-- Ignition off - show open-circuit voltage
		gaugeVoltage = is24V and 75 or 100
	end

	-- Initialize voltage smoother if not exists
	if not device.voltsSmoother then
		device.voltsSmoother = newExponentialSmoothing(7) -- Same smoothing factor as GaugesDaddy
	end

	-- Apply smoothing to gauge voltage
	local smoothedVoltage = device.voltsSmoother:get(gaugeVoltage, dt)

	-- Store smoothed voltage for gauge display
	device.gaugeVoltage = smoothedVoltage

	-- Update electrics with the smoothed gauge voltage and engine reference
	electrics.values.volts = batteryVoltage -- Use raw voltage for standard gauges
	if not electrics.values.engine or type(electrics.values.engine) ~= "table" then
		electrics.values.engine = {}
	end
	electrics.values.engine.batteryVoltage = batteryVoltage
	electrics.values.engine.starterCurrent = starterCurrent
	electrics.values.engine.alternatorVoltage = isEngineRunning and maxVoltage or 0
	electrics.values.batteryVoltage = batteryVoltage -- Provide raw voltage
	electrics.values.Volts = batteryVoltage -- Provide raw voltage to capitalized Volts as well
	electrics.values.voltsScaled = smoothedVoltage -- Keep scaled version for unique mods

	-- Sync amps (usually positive is charge, negative is discharge)
	local currentDraw = (isEngineRunning and 0 or -5) -- Nominal discharge when running/idle
	if device.starterEngagedCoef > 0 then
		currentDraw = -starterCurrent -- Discharge heavily when cranking
	elseif isEngineRunning then
		currentDraw = 15 -- Typical alternator charging current
	end
	electrics.values.amps = currentDraw
	electrics.values.Amps = currentDraw
	electrics.values.batteryCharge = device.batteryCharge

	-- Update battery warning light based on voltage (similar to GaugesDaddy)
	if device.gaugeVoltage < (is24V and 75 or 85) and (device.ignitionCoef or 0) > 0.5 then
		electrics.values.battery = 1 -- Warning light on
	else
		electrics.values.battery = 0 -- Warning light off
	end

	-- Update fuel properties periodically (e.g., every 100 frames)
	device.fuelUpdateTimer = (device.fuelUpdateTimer or 0) + 1
	--[[if device.fuelUpdateTimer >= 100 then
	    device.currentFuelProperties = checkFuelCompatibility(device)
		device.fuelEffectFactor = device.currentFuelProperties.energyDensity

		-- Check for fuel compatibility and define cumulative damage rates
		local fuel = device.currentFuelProperties
		local reqType = device.requiredEnergyType

		if reqType == "gasoline" then
			if fuel.category == "gasoline" then
				local reqOctane = device.requiredOctane or 91
				if fuel.octaneRating < reqOctane then
					-- Octane too low: cumulative damage (knock/pre-ignition)
					device.fuelDamageRate = 0.002 * (reqOctane - fuel.octaneRating)
				else
					device.fuelDamageRate = 0
				end
			elseif fuel.category == "ethanol" then
				-- Ethanol in gasoline engine: potential lean condition damage
				device.fuelDamageRate = 0.005
			elseif fuel.category == "diesel" then
				-- Diesel in gasoline engine: major fouling and severe damage
				device.fuelDamageRate = 0.04
			end
		elseif reqType == "diesel" then
			if fuel.category == "diesel" then
				device.fuelDamageRate = 0
			elseif fuel.category == "gasoline" or fuel.category == "ethanol" then
				-- Gasoline/Ethanol in diesel: catastrophic high-pressure pump/injector damage
				device.fuelDamageRate = 0.1 -- extremely rapid failure
			end
		end
		device.fuelUpdateTimer = 0
	end]]

	-- Apply fuel-based damage to engine integrity
	if device.fuelDamageRate > 0 and isEngineRunning then
		device.damageFrictionCoef = device.damageFrictionCoef + device.fuelDamageRate * dt
		if (device.lastDamageLogTime or 0) + 5 < (device.time or 0) then
			log("W", "combustionEngine.fuelDamage", "Engine suffering damage due to incompatible fuel!")
			device.lastDamageLogTime = device.time
		end
	end

	-- change to true to enable debugging logs
	local debugBatt = false

	if debugBatt then
		-- log detailed battery state every 50 physics ticks when starter is engaged, or every 200 ticks when not
		if device.starterEngagedCoef > 0 or isEngineRunning or device.batteryLogCounter % 800 == 0 then
			if device.batteryLogCounter % 1000 == 0 then
				local currentVoltage = batteryVoltage
				local starterTorque = device.starterTorque
					or (isDiesel and 18.95 or 17.86)
						* device.starterMaxAV
						* (tempEffect or 1.0)
						* (voltageEffect or 1.0)

				-- Debug print to console
				print(
					string.format(
						"[Battery] Debug - starterEngagedCoef: %.2f, voltageFactor: %.2f, starterCurrent: %.2f",
						device.starterEngagedCoef,
						batteryVoltageFactor,
						starterCurrent
					)
				)

				-- Detailed GUI message
				-- Calculate additional battery metrics
				local stateOfCharge =
					math.max(0, math.min(1, (batteryVoltage - minVoltage) / (maxVoltage - minVoltage)))
				local starterPower = starterCurrent * batteryVoltage / 1000 -- In kW

				local logMsg = string.format(
					"Battery State:\n"
						.. "Voltage: %.1fV (%.0f%% SOC)\n"
						.. "System: %s | Charge: %.0f%%\n"
						.. "Current: %.1fA | Power: %.1fkW\n"
						.. "Temp Effect: %.2fx | Load: %.0f%%\n"
						.. "Starter Torque: %.1f Nm | RPM: %.0f\n"
						.. "Counter: %d | Time: %.1fs",
					batteryVoltage,
					stateOfCharge * 100,
					is24V and "24V" or "12V",
					chargeLevel * 100,
					starterCurrent,
					starterPower,
					tempEffect or 1.0,
					(1.0 - (device.outputAV1 / (device.starterMaxAV * 0.5))) * 100,
					starterTorque,
					device.outputAV1 * 9.5493, -- Convert rad/s to RPM
					device.batteryLogCounter,
					device.batteryLogCounter * 0.0167 -- Approximate time in seconds (60 ticks per second)
				)

				-- Send to both console and GUI for visibility
				print("[Battery] " .. logMsg:gsub("\n", " | "))

				-- Show battery status message when voltage is low or when starter is engaged
				local currentVoltage = device.batteryNominalVoltage * device.batteryCharge
				if currentVoltage < device.batteryWarningVoltage or device.starterEngagedCoef > 0 then
					local battStatus = string.format(
						"Battery Status:\n" .. "Voltage: %.1fV / %.1fV\n" .. "Charge: %d%%\n" .. "Starter: %s",
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
	-- Initialize starting hesitation system if not already done
	if hesitationDebug then
		log("D", "startingHesitation", "Starting hesitation update for device: " .. (device.name or "unknown"))
	end
	if not device.startingHesitationInitialized then
		device.startingHesitationInitialized = true
		device.startingHesitationTime = 0
		device.startingHesitationPhase = 0 -- 0=initial crank, 1=struggle, 2=normal cranking, 3=running
		device.startingHesitationFactor = 1.0
		device.lastStarterState = false
	end

	-- Check if we just started cranking
	local isCranking = device.starterEngagedCoef > 0.1
	local justStartedCranking = isCranking and not device.lastStarterState
	device.lastStarterState = isCranking

	if justStartedCranking then
		-- Reset hesitation state when starting to crank
		device.startingHesitationPhase = 0
		device.startingHesitationTime = 0
		device.startingHesitationFactor = 1.0
	end

	-- Track starting hesitation effect
	if isCranking and not isEngineRunning then
		-- Update timer for hesitation effect
		device.startingHesitationTime = device.startingHesitationTime + dt

		-- Stage 0: Initial Surge (Short, strong kick to overcome inertia)
		if device.startingHesitationPhase == 0 then
			local initialCrankDuration = 0.2 + math.random() * 0.4 -- 0.6s to 1.0s surge
			device.startingHesitationFactor = 1.3 -- Strong initial torque
			if device.startingHesitationTime > initialCrankDuration then
				device.startingHesitationPhase = 1 -- Move to struggle phase
				if hesitationDebug then
					log(
						"D",
						"startingHesitation",
						"Phase changed to struggle for device: " .. (device.name or "unknown")
					)
				end
				device.startingHesitationTime = 0
			end

			-- Phase 1: Struggle phase (extended duration)
		elseif device.startingHesitationPhase == 1 then
			local coldFactor = math.max(0, math.min(1, (20 - (engineTempC or 20)) / 40)) -- 0 at 20°C, 1 at -20°C
			local struggleDuration = 3.0 + coldFactor * 3.0 + math.random() * 1.0 -- 3-4s warm, up to 7s very cold

			-- Create a pulsing effect during struggle
			local pulseFreq = math.random(1.0, 30.0) -- random frequency between 1 and 30 Hz
			local pulse = (math.sin(device.startingHesitationTime * math.pi * 8 * pulseFreq) + 1) * 0.5 -- 0-1 pulse

			-- Base hesitation effect (lower torque during struggle)
			device.startingHesitationFactor = 0.7 + pulse * 0.25 -- 70-95% torque
			if hesitationDebug then
				log(
					"D",
					"startingHesitation",
					"Struggle phase factor: "
						.. device.startingHesitationFactor
						.. " for device: "
						.. (device.name or "unknown")
				)
			end

			-- Add occasional stronger hesitation (misfires)
			if math.random() < 0.1 then
				device.startingHesitationFactor = device.startingHesitationFactor * (0.2 + math.random() * 0.1) -- 20-30% of current factor
				if hesitationDebug then
					log(
						"D",
						"startingHesitation",
						"Misfire occurred, factor reduced to: "
							.. device.startingHesitationFactor
							.. " for device: "
							.. (device.name or "unknown")
					)
				end
			end

			-- Transition to normal cranking after struggle duration
			if device.startingHesitationTime > struggleDuration then
				device.startingHesitationPhase = 2
				if hesitationDebug then
					log(
						"D",
						"startingHesitation",
						"Phase changed to normal cranking for device: " .. (device.name or "unknown")
					)
				end
				device.startingHesitationTime = 0
			end

			-- Phase 2: Normal cranking (full power)
		elseif device.startingHesitationPhase == 2 then
			device.startingHesitationFactor = 1.0 -- Full power
		end

		-- Reset when not cranking or when engine is running
	else
		if isEngineRunning then
			device.startingHesitationPhase = 3 -- Engine is running
			if hesitationDebug then
				log(
					"D",
					"startingHesitation",
					"Engine is running, phase set to 3 for device: " .. (device.name or "unknown")
				)
			end
			device.startingHesitationFactor = 0.0
		else
			device.startingHesitationFactor = 1.0
			if hesitationDebug then
				log(
					"D",
					"startingHesitation",
					"Not cranking, factor set to 1.0 for device: " .. (device.name or "unknown")
				)
			end
		end
	end

	-- Track engine state for misfire triggers (Flood recovery logic handled globally above)

	-- Initialize ignition error timers/states/coefficients (Always run to prevent nil crashes)
	device.slowIgnitionErrorTimer = device.slowIgnitionErrorTimer or 0
	device.fastIgnitionErrorTimer = device.fastIgnitionErrorTimer or 0
	device.starterIgnitionErrorTimer = device.starterIgnitionErrorTimer or 0
	device.slowIgnitionErrorActive = device.slowIgnitionErrorActive or false
	device.fastIgnitionErrorActive = device.fastIgnitionErrorActive or false
	device.starterIgnitionErrorActive = device.starterIgnitionErrorActive or false
	device.slowIgnitionErrorCoef = device.slowIgnitionErrorCoef or 1
	device.fastIgnitionErrorCoef = device.fastIgnitionErrorCoef or 1
	device.starterIgnitionErrorCoef = device.starterIgnitionErrorCoef or 1
	device.slowIgnitionErrorDuration = device.slowIgnitionErrorDuration or 0
	device.fastIgnitionErrorDuration = device.fastIgnitionErrorDuration or 0
	device.starterIgnitionErrorDuration = device.starterIgnitionErrorDuration or 0
	device.slowIgnitionErrorInterval = device.slowIgnitionErrorInterval or 5

	-- Apply starter-specific effects
	if device.starterEngagedCoef > 0 then
		-- Logic for starter-specific pulsing is handled further down via baseStarterTorque mapping
	end

	-- Get number of cylinders and update cycles (Always run while engine is active)
	local cylinderCount = device.fundamentalFrequencyCylinderCount or jbeamData.cylinderCount or 8
	device.cyclePosition = (device.cyclePosition or 0) + (math.abs(device.outputAV1) * dt)
	device.cyclePosition = device.cyclePosition % (4 * math.pi)

	-- Update cylinder states based on cycle position
	local cyclePosPerCylinder = (4 * math.pi) / cylinderCount
	local currentCylinder = math.floor(device.cyclePosition / cyclePosPerCylinder) + 1

	-- Define base torque reference for failure subtractions
	local baseCombustionTorque = (device.torqueCurve[floor(math.abs(device.outputAV1) * avToRPM)] or 0)
		* (device.intakeAirDensityCoef or 1.0)

	-- Update each cylinder's state based on its position in the cycle
	for i = 1, cylinderCount do
		local cylinder = device.cylinders[i]
		-- Offset each cylinder by its position in the firing order (0 to 4*pi)
		local cylinderOffset = (i - 1) * cyclePosPerCylinder
		local cyclePos = (device.cyclePosition - cylinderOffset) % (4 * math.pi)

		-- Determine stroke (0-3: intake, compression, power, exhaust)
		-- Each stroke is 180 degrees (pi radians)
		local stroke = math.floor(cyclePos / math.pi) % 4

		-- Update cylinder state based on stroke
		cylinder.isCompressing = (stroke == 1)
		cylinder.isFiring = (stroke == 2)

		-- Handle fuel injection during intake stroke
		-- Reduced throttle gate to allow idle fueling (State-based gate instead)
		if stroke == 0 and not cylinder.isFiring and (isStarting or isRunning or throttle > 0) then
			-- Check if we should be adding fuel
			local timeSinceLastFuel = (device.cyclePosition - (cylinder.lastFuelAddTime or -10))
			local isReadyForFuel = timeSinceLastFuel > (isCranking and 0.47 or 0.2) -- More frequent injection when cranking

			if (isStarting or isRunning) and isReadyForFuel then
				-- Calculate base fuel amount with all enrichment factors
				local fuelAmount = baseFuelAmount * throttle
				
				-- Ensure minimum fuel at low throttle to prevent lean misfires
				-- Especially important during cold starts and idle
				if throttle < 0.1 then
					fuelAmount = math.max(fuelAmount, baseFuelAmount * 0.15)  -- Minimum 15% of base fuel at very low throttle
				end

				-- Apply cranking enrichment when cranking (reduced from original)
				if isCranking then
					local crankingMultiplier = device.carburetor and device.carburetor.constants.crankingFuelMultiplier
						or 1.5
					fuelAmount = fuelAmount * (crankingMultiplier * 0.7) -- Reduced by 30%

					-- Add smaller extra fuel pulse at the beginning of cranking
					if device.cyclePosition < (2 * math.pi) then -- First revolution
						fuelAmount = fuelAmount * 1.2 -- Reduced from 1.5
					end
				end

				-- Apply choke enrichment (further reduced)
				fuelAmount = fuelAmount * (1.0 + (device.chokeEffect * 0.2)) -- Reduced to 20% max extra fuel with choke

				-- Use global flood level for all cylinders
				local cylinderFlood = device.floodLevel

				-- Reduce fuel if cylinder is flooded (more forgiving thresholds)
				if cylinderFlood > 0.8 then -- Increased threshold from 0.7
					-- Reduce fuel significantly but don't cut completely
					fuelAmount = fuelAmount * 0.3 -- Reduced from 0 to 0.3 (30% fuel)

					-- Better chance to clear some flood when fuel is reduced
					if device.idleAV > 0.15 then -- Increased from 0.1
						device.floodLevel = math.max(0, cylinderFlood - 0.15) -- Increased from 0.1
					end
				elseif cylinderFlood > 0.3 then -- Increased threshold from 0.3
					-- More progressive fuel reduction for partially flooded cylinder
					fuelAmount = fuelAmount * (1.0 - cylinderFlood * 1.5) -- Less aggressive reduction
				end

				-- Ensure minimum fuel injection amount
				fuelAmount = math.max(fuelAmount, minFuelForInjection)
				
				-- CRITICAL FIX: Ensure fuel meets minimum combustion requirements
				-- At cold temps, minFuelForCombustion is higher than minFuelForInjection
				-- This prevents misfires due to insufficient fuel for combustion
				fuelAmount = math.max(fuelAmount, minFuelForCombustion * 0.9)  -- 90% of min for combustion to allow some leanness

				-- Add fuel to cylinder with a minimum amount
				local newFuel = math.min(maxFuelPerCylinder, cylinder.fuelAmount + fuelAmount * dt * 0.8)
				local debugFuel = false

				-- Debug output for fuel addition with rate limiting
				if debugFuel and fuelAmount > 0 then
					device.lastFuelLogTime = device.lastFuelLogTime or {}
					device.lastFuelLogTime[i] = device.lastFuelLogTime[i] or 0
					local currentTime = os.clock()

					if currentTime - device.lastFuelLogTime[i] > 1.0 then -- Limit to once per second per cylinder
						print(string.format(
							"[FUEL] Cyl %d: Adding %.6f (total: %.6f) at pos %.2f, RPM: %.1f, Throttle: %.2f, State: %s, Temp: %.2f (%.1fs)",
							i,
							fuelAmount,
							newFuel,
							device.cyclePosition,
							math.abs(device.outputAV1) * (30 / math.pi), -- Convert rad/s to RPM
							throttle,
							isCranking and "CRANKING" or (isRunning and "RUNNING" or "STARTING"),
							engineTempC,
							currentTime
						))
						device.lastFuelLogTime[i] = currentTime
					end
				end
				-- Apply side-effects of high fuel pressure (increased flooding risk)
				if device.fuelPressureMultiplier > 1.2 then
					local floodInc = (device.fuelPressureMultiplier - 1.2) * 0.005 * dt
					device.floodLevel = math.min(1.0, device.floodLevel + floodInc)

					-- Chance to blow an injector at extreme pressure
					if device.fuelPressureMultiplier > 1.8 and math.random() < 0.0001 then
						cylinder.failMode = "broken"
						log(
							"E",
							"combustionEngine.failure",
							"Extreme fuel pressure caused an injector to fail on cylinder " .. i
						)
					end
				end

				-- Handle compression loss based on failure mode
				if cylinder.failMode == "dead" then
					cylinder.failAirScale = 0.15
				elseif cylinder.failMode == "broken" then
					cylinder.failAirScale = 0.01
				elseif cylinder.failMode == "leak" then
					-- Gradually reduce compression for leaky cylinders
					cylinder.failAirScale = math.max(0.2, (cylinder.failAirScale or 1.0) - (0.1 * dt))
				else
					cylinder.failAirScale = 1.0
				end

				cylinder.fuelAmount = newFuel * device.fuelPressureMultiplier
				cylinder.airAmount = (device.intakeAirDensityCoef or 0.8) * cylinder.failAirScale

				-- Side effect of air restriction: increased thermal load
				if device.airRestrictionMulti