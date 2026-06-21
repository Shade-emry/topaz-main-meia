/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022-2026 SlimeVR Contributors
*/

#include "GyroTemperatureCalibrator.h"

#include <algorithm>
#include <cmath>

#include "GlobalVars.h"

void GyroTemperatureCalibrator::resetCurrentTemperatureState() {
	state.numSamples = 0;
	state.tSum = 0;
	state.xSum = 0;
	state.ySum = 0;
	state.zSum = 0;
}

bool GyroTemperatureCalibrator::updateGyroTemperatureCalibration(
	const float temperature,
	const bool restDetected,
	int32_t x,
	int32_t y,
	int32_t z
) {
	if (!calibrationRunning || !std::isfinite(temperature)) {
		return false;
	}
	if (!restDetected) {
		resetCurrentTemperatureState();
		return false;
	}

	const int16_t idx = TEMP_CALIBRATION_TEMP_TO_IDX(temperature);
	if (idx < 0 || idx >= TEMP_CALIBRATION_BUFFER_SIZE) {
		return false;
	}

	const float target = TEMP_CALIBRATION_IDX_TO_TEMP(idx);
	if (std::fabs(temperature - target)
		> TEMP_CALIBRATION_MAX_DEVIATION_FROM_STEP) {
		resetCurrentTemperatureState();
		return false;
	}

	if (state.temperatureCurrentIdx != static_cast<uint16_t>(idx)) {
		state.temperatureCurrentIdx = idx;
		resetCurrentTemperatureState();
		if (!backgroundLearning) {
			m_Logger.info(
				"[TEMPCAL] Stationary acquisition started at %.2f C; do not move "
				"for %.0f seconds",
				temperature,
				TEMP_CALIBRATION_SECONDS_PER_STEP
			);
		}
	}

	state.numSamples++;
	state.tSum += temperature;
	state.xSum += x;
	state.ySum += y;
	state.zSum += z;
	if (state.numSamples < samplesPerStep) {
		return false;
	}

	GyroTemperatureOffsetSample measured;
	measured.t = state.tSum / state.numSamples;
	measured.x = static_cast<double>(state.xSum) / state.numSamples;
	measured.y = static_cast<double>(state.ySum) / state.numSamples;
	measured.z = static_cast<double>(state.zSum) / state.numSamples;

	auto& existing = config.samples[idx];
	auto& observations = config.observations[idx];
	if (observations == 0) {
		existing = measured;
		observations = 1;
		config.samplesTotal++;
	} else {
		const float weight = static_cast<float>(std::min<uint16_t>(observations, 31));
		existing.t = (existing.t * weight + measured.t) / (weight + 1.0f);
		existing.x = (existing.x * weight + measured.x) / (weight + 1.0f);
		existing.y = (existing.y * weight + measured.y) / (weight + 1.0f);
		existing.z = (existing.z * weight + measured.z) / (weight + 1.0f);
		if (observations < UINT16_MAX) {
			observations++;
		}
	}

	config.minTemperatureRange
		= std::min(config.minTemperatureRange, existing.t);
	config.maxTemperatureRange
		= std::max(config.maxTemperatureRange, existing.t);
	config.minCalibratedIdx
		= TEMP_CALIBRATION_TEMP_TO_IDX(config.minTemperatureRange);
	config.maxCalibratedIdx
		= TEMP_CALIBRATION_TEMP_TO_IDX(config.maxTemperatureRange);
	config.hasCoeffs = false;
	dirty = true;
	acceptedSinceSave++;
	lastApproximatedTemperature = 0;

	m_Logger.info(
		"[TEMPCAL] Accepted %d C bin (%u observations); curve %u/%u bins",
		static_cast<int>(target),
		observations,
		config.samplesTotal,
		TEMP_CALIBRATION_BUFFER_SIZE
	);
	if (!backgroundLearning) {
		m_Logger.info("[TEMPCAL] You may move the tracker");
	}

	resetCurrentTemperatureState();
	if (acceptedSinceSave >= 5) {
		saveConfig();
	}
	return true;
}

bool GyroTemperatureCalibrator::approximateOffset(
	const float temperature,
	float GOxyz[3]
) {
	if (!config.hasData() || !std::isfinite(temperature)) {
		return false;
	}
	if (lastApproximatedTemperature != 0.0f
		&& std::fabs(temperature - lastApproximatedTemperature) < 0.001f) {
		for (size_t axis = 0; axis < 3; axis++) {
			GOxyz[axis] = lastApproximatedOffsets[axis];
		}
		return true;
	}

	const float constrainedTemperature = constrain(
		temperature,
		config.minTemperatureRange,
		config.maxTemperatureRange
	);
	int lower = TEMP_CALIBRATION_TEMP_TO_IDX(constrainedTemperature);
	lower = std::clamp(lower, 0, static_cast<int>(TEMP_CALIBRATION_BUFFER_SIZE) - 1);
	int upper = lower;
	while (lower >= 0 && config.observations[lower] == 0) {
		lower--;
	}
	while (upper < TEMP_CALIBRATION_BUFFER_SIZE
		   && config.observations[upper] == 0) {
		upper++;
	}
	if (lower < 0 && upper >= TEMP_CALIBRATION_BUFFER_SIZE) {
		return false;
	}
	if (lower < 0) {
		lower = upper;
	}
	if (upper >= TEMP_CALIBRATION_BUFFER_SIZE) {
		upper = lower;
	}

	const auto& low = config.samples[lower];
	const auto& high = config.samples[upper];
	float interpolation = 0;
	if (upper != lower && std::fabs(high.t - low.t) > 0.01f) {
		interpolation = std::clamp(
			(constrainedTemperature - low.t) / (high.t - low.t),
			0.0f,
			1.0f
		);
	}
	GOxyz[0] = low.x + interpolation * (high.x - low.x);
	GOxyz[1] = low.y + interpolation * (high.y - low.y);
	GOxyz[2] = low.z + interpolation * (high.z - low.z);

	lastApproximatedTemperature = temperature;
	for (size_t axis = 0; axis < 3; axis++) {
		lastApproximatedOffsets[axis] = GOxyz[axis];
	}
	return true;
}

bool GyroTemperatureCalibrator::loadConfig(float newSensitivity) {
	const bool ok = configuration.loadTemperatureCalibration(sensorId, config);
	if (!ok || config.version != 2 || !config.hasData()) {
		m_Logger.info("[TEMPCAL] No compatible saved curve; learning is available");
		return false;
	}
	config.rescaleSamples(newSensitivity);
	configSaved = true;
	m_Logger.info(
		"[TEMPCAL] Loaded %u/%u bins covering %.1f-%.1f C",
		config.samplesTotal,
		TEMP_CALIBRATION_BUFFER_SIZE,
		config.minTemperatureRange,
		config.maxTemperatureRange
	);
	return true;
}

bool GyroTemperatureCalibrator::saveConfig() {
	if (!dirty && configSaved) {
		return true;
	}
	if (configuration.saveTemperatureCalibration(sensorId, config)) {
		configSaved = true;
		configSaveFailed = false;
		dirty = false;
		acceptedSinceSave = 0;
		m_Logger.info(
			"[TEMPCAL] Curve saved: %u/%u bins, range %.1f-%.1f C",
			config.samplesTotal,
			TEMP_CALIBRATION_BUFFER_SIZE,
			config.hasData() ? config.minTemperatureRange : 0.0f,
			config.hasData() ? config.maxTemperatureRange : 0.0f
		);
		return true;
	}
	configSaveFailed = true;
	m_Logger.error("[TEMPCAL] Failed to save curve");
	return false;
}

void GyroTemperatureCalibrator::startLearning(bool background) {
	backgroundLearning = background;
	calibrationRunning = true;
	resetCurrentTemperatureState();
	m_Logger.info(
		"[TEMPCAL] Learning started%s",
		background ? " in background mode" : ""
	);
	m_Logger.info(
		"[TEMPCAL] Target %.0f-%.0f C, %.0f C bins; keep the tracker still when "
		"acquisition starts",
		TEMP_CALIBRATION_MIN,
		TEMP_CALIBRATION_MAX,
		TEMP_CALIBRATION_STEP
	);
	m_Logger.info(
		"[TEMPCAL] Existing curve: %u/%u bins",
		config.samplesTotal,
		TEMP_CALIBRATION_BUFFER_SIZE
	);
}

void GyroTemperatureCalibrator::stopLearning(bool save) {
	calibrationRunning = false;
	resetCurrentTemperatureState();
	if (save) {
		saveConfig();
	}
	m_Logger.info("[TEMPCAL] Learning stopped");
}

void GyroTemperatureCalibrator::setBackgroundLearning(bool enabled) {
	if (enabled) {
		startLearning(true);
	} else {
		stopLearning(true);
	}
}

void GyroTemperatureCalibrator::printStatus() const {
	m_Logger.info(
		"[TEMPCAL] %s, background=%s, curve=%u/%u bins, range %.1f-%.1f C",
		calibrationRunning ? "learning" : "idle",
		backgroundLearning ? "on" : "off",
		config.samplesTotal,
		TEMP_CALIBRATION_BUFFER_SIZE,
		config.hasData() ? config.minTemperatureRange : 0.0f,
		config.hasData() ? config.maxTemperatureRange : 0.0f
	);
}

void GyroTemperatureCalibrator::printSamples() const {
	printStatus();
	for (uint16_t i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
		if (config.observations[i] == 0) {
			continue;
		}
		const auto& sample = config.samples[i];
		m_Logger.info(
			"[TEMPCAL] %.1f C: %.3f %.3f %.3f raw (%u observations)",
			sample.t,
			sample.x,
			sample.y,
			sample.z,
			config.observations[i]
		);
	}
}

void GyroTemperatureCalibrator::clear() {
	config.reset();
	configSaved = false;
	dirty = true;
	lastApproximatedTemperature = 0;
	saveConfig();
	m_Logger.info("[TEMPCAL] Curve cleared");
}
