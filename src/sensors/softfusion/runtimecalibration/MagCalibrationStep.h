/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors
*/

#pragma once

#include <magneto1.4.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "CalibrationStep.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename SensorRawT>
class MagCalibrationStep : public CalibrationStep<SensorRawT> {
public:
	using Base = CalibrationStep<SensorRawT>;
	using TickResult = typename Base::TickResult;

	MagCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig
	)
		: Base{sensorConfig} {}

	void start() final {
		Base::start();
		calibration = MagnetoCalibration{};
		startMillis = millis();
		sampleCount = 0;
		for (size_t axis = 0; axis < 3; axis++) {
			minimum[axis] = std::numeric_limits<float>::infinity();
			maximum[axis] = -std::numeric_limits<float>::infinity();
		}
	}

	TickResult tick() final {
		if (millis() - startMillis < CalibrationDurationMillis
			|| sampleCount < MinimumSamples || !hasSufficientCoverage()) {
			return TickResult::CONTINUE;
		}

		float result[4][3];
		calibration.current_calibration(result);
		for (size_t row = 0; row < 4; row++) {
			for (size_t column = 0; column < 3; column++) {
				if (!std::isfinite(result[row][column])) {
					start();
					return TickResult::CONTINUE;
				}
			}
		}

		for (size_t axis = 0; axis < 3; axis++) {
			this->sensorConfig.M_B[axis] = result[0][axis];
			this->sensorConfig.M_Ainv[0][axis] = result[1][axis];
			this->sensorConfig.M_Ainv[1][axis] = result[2][axis];
			this->sensorConfig.M_Ainv[2][axis] = result[3][axis];
		}
		this->sensorConfig.magCalibrated = true;
		return TickResult::DONE;
	}

	void cancel() final {}
	bool requiresRest() final { return false; }

	void processMagSample(const SensorRawT sample[3]) final {
		calibration.sample(sample[0], sample[1], sample[2]);
		for (size_t axis = 0; axis < 3; axis++) {
			const float value = static_cast<float>(sample[axis]);
			minimum[axis] = std::min(minimum[axis], value);
			maximum[axis] = std::max(maximum[axis], value);
		}
		sampleCount++;
	}

private:
	bool hasSufficientCoverage() const {
		float largestSpan = 0;
		float smallestSpan = std::numeric_limits<float>::infinity();
		for (size_t axis = 0; axis < 3; axis++) {
			const float span = maximum[axis] - minimum[axis];
			largestSpan = std::max(largestSpan, span);
			smallestSpan = std::min(smallestSpan, span);
		}
		return largestSpan > 0 && smallestSpan >= largestSpan * MinimumSpanRatio;
	}

	static constexpr uint32_t CalibrationDurationMillis = 30000;
	static constexpr uint32_t MinimumSamples = 100;
	static constexpr float MinimumSpanRatio = 0.15f;

	MagnetoCalibration calibration;
	uint32_t startMillis = 0;
	uint32_t sampleCount = 0;
	float minimum[3]{};
	float maximum[3]{};

	using Base::sensorConfig;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
