/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "../../../GlobalVars.h"
#include "../../../calibration.h"
#include "../../../configuration/Configuration.h"
#include "../../../motionprocessing/GyroTemperatureCalibrator.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MagCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/axisremap.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = false;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles}
		, temperatureCalibrator(
			  SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION,
			  sensorId,
			  IMU::GyroSensitivity,
			  static_cast<uint32_t>(
				  TEMP_CALIBRATION_SECONDS_PER_STEP / IMU::GyrTs
			  )
		  ) {
		calibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.runtimeCalibration.ImuType == IMU::Type)
			&& (sensorCalibration.data.runtimeCalibration.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.runtimeCalibration;
		sanitizeCalibration(calibration);
		refreshActiveCalibration();
		calculateZROChange();

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();
		startupSequenceComplete = false;

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();
		refreshActiveCalibration();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();

#if ENABLE_GYRO_TEMP_CURVE
		temperatureCalibrator.loadConfig(IMU::GyroSensitivity);
#endif

		printCalibration();
		if (!hasRequiredCalibration()) {
			logger.info("[CAL] First-start calibration is required before tracking");
			logger.info("[CAL] Keep the tracker still for gyro calibration");
			if (!accelBiasCalibrationStep.allSidesCalibrated()) {
				logger.info(
					"[CAL] Accelerometer: place each of the six faces level and still "
					"when prompted"
				);
			}
		} else {
			logger.info(
				"[CAL] Saved calibration loaded; tracking will start after the "
				"stationary startup gyro check"
			);
		}
	}

	void setMagnetometerAvailable(bool available) final {
		magnetometerAvailable = available;
		if (available && !calibration.magCalibrated) {
			logger.info(
				"[CAL] Magnetometer: rotate the tracker through all axes when prompted"
			);
		}
	}

	[[nodiscard]] bool isStartupCalibrationComplete() const final {
		return startupSequenceComplete && hasRequiredCalibration();
	}

	[[nodiscard]] bool hasRequiredCalibration() const {
		const bool motionlessReady
			= !Base::HasMotionlessCalib || calibration.motionlessCalibrated;
		const bool magReady = !magnetometerAvailable || calibration.magCalibrated;
		return calibration.sensorTimestepsCalibrated && motionlessReady
			&& calibration.gyroPointsCalibrated > 0
			&& accelBiasCalibrationStep.allSidesCalibrated() && magReady;
	}

	void startCalibration(int calibrationType) final {
		currentStep->cancel();
		startupSequenceComplete = false;
		switch (calibrationType) {
			case CALIBRATION_TYPE_INTERNAL_GYRO:
			case CALIBRATION_TYPE_EXTERNAL_GYRO:
				currentStep = &gyroBiasCalibrationStep;
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				logger.info("[CAL] Gyro calibration requested; keep tracker still");
				break;
			case CALIBRATION_TYPE_INTERNAL_ACCEL:
			case CALIBRATION_TYPE_EXTERNAL_ACCEL:
				std::fill(
					std::begin(calibration.accelCalibrated),
					std::end(calibration.accelCalibrated),
					false
				);
				std::fill(
					std::begin(calibration.accelSideCalibrated),
					std::end(calibration.accelSideCalibrated),
					false
				);
				currentStep = &accelBiasCalibrationStep;
				nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
				logger.info(
					"[CAL] Accelerometer calibration requested; place all six faces "
					"level and still in turn"
				);
				break;
			case CALIBRATION_TYPE_INTERNAL_MAG:
			case CALIBRATION_TYPE_EXTERNAL_MAG:
				calibration.magCalibrated = false;
				currentStep = &magCalibrationStep;
				nextCalibrationStep = CalibrationStepEnum::MAG;
				logger.info(
					"[CAL] Magnetometer calibration requested; rotate through all axes "
					"for at least 30 seconds"
				);
				break;
			default:
				logger.warn(
					"Runtime calibration type %d is not supported",
					calibrationType
				);
				return;
		}
		isCalibrating = false;
	}

	void cancelCalibration() final {
		currentStep->cancel();
		currentStep = &nullCalibrationStep;
		nextCalibrationStep = CalibrationStepEnum::NONE;
		isCalibrating = false;
		startupSequenceComplete = hasRequiredCalibration();
		logger.info("[CAL] Calibration cancelled");
	}

	void printCalibrationStatus() final {
		printCalibration();
		logger.info(
			"[CAL] Startup ready: %s",
			isStartupCalibrationComplete() ? "yes" : "no"
		);
	}

	void tick() final {
		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		accelSample[0] = (accelSample[0] * Consts::AScale
						  - activeCalibration.A_off[0])
					   * activeCalibration.A_scale[0];
		accelSample[1] = (accelSample[1] * Consts::AScale
						  - activeCalibration.A_off[1])
					   * activeCalibration.A_scale[1];
		accelSample[2] = (accelSample[2] * Consts::AScale
						  - activeCalibration.A_off[2])
					   * activeCalibration.A_scale[2];
		remapAllAxis(
			activeCalibration.imuAxisRemap,
			&accelSample[0],
			&accelSample[1],
			&accelSample[2]
		);
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		float gyroOffset[3]{};
		getGyroOffset(gyroOffset);

		gyroSample[0] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[0] - gyroOffset[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[1] - gyroOffset[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[2] - gyroOffset[2])
		);
		remapAllAxis(
			activeCalibration.imuAxisRemap,
			&gyroSample[0],
			&gyroSample[1],
			&gyroSample[2]
		);
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }

	void scaleMagSample(sensor_real_t magSample[3]) final {
		const sensor_real_t unbiased[3]{
			magSample[0] - activeCalibration.M_B[0],
			magSample[1] - activeCalibration.M_B[1],
			magSample[2] - activeCalibration.M_B[2],
		};

		for (size_t row = 0; row < 3; row++) {
			magSample[row] = activeCalibration.M_Ainv[row][0] * unbiased[0]
						   + activeCalibration.M_Ainv[row][1] * unbiased[1]
						   + activeCalibration.M_Ainv[row][2] * unbiased[2];
		}
		remapAllAxis(
			activeCalibration.magAxisRemap,
			&magSample[0],
			&magSample[1],
			&magSample[2]
		);
	}

	float getMagTimestep() final { return activeCalibration.M_Ts; }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
#if ENABLE_GYRO_TEMP_CURVE
		if (std::isfinite(currentTemperature)
			&& temperatureCalibrator.isCalibrating()) {
			float oldOffset[3]{};
			getGyroOffset(oldOffset);
			const bool accepted
				= temperatureCalibrator.updateGyroTemperatureCalibration(
					currentTemperature,
					fusion.getRestDetected(),
					gyroSample[0],
					gyroSample[1],
					gyroSample[2]
				);
			if (accepted) {
				float newOffset[3]{};
				getGyroOffset(newOffset);
				sensor_real_t delta[3]{
					static_cast<sensor_real_t>(
						(newOffset[0] - oldOffset[0]) * Consts::GScale
					),
					static_cast<sensor_real_t>(
						(newOffset[1] - oldOffset[1]) * Consts::GScale
					),
					static_cast<sensor_real_t>(
						(newOffset[2] - oldOffset[2]) * Consts::GScale
					),
				};
				remapAllAxis(
					activeCalibration.imuAxisRemap,
					&delta[0],
					&delta[1],
					&delta[2]
				);
				fusion.transferGyroPreBias(delta);
			}
		}
#endif
	}

	void provideTempSample(float tempSample) final {
		currentTemperature = tempSample;
		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void provideMagSample(const RawSensorT magSample[3]) final {
		if (isCalibrating) {
			currentStep->processMagSample(magSample);
		}
	}

	void calculateZROChange() {
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
			return;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		const float temperatureSpan
			= activeCalibration.gyroMeasurementTemperature2
			- activeCalibration.gyroMeasurementTemperature1;
		if (maxDiff <= std::numeric_limits<float>::epsilon()
			|| std::abs(temperatureSpan) <= MinimumTemperatureSpan) {
			activeZROChange = IMU::TemperatureZROChange;
			return;
		}

		activeZROChange = 0.1f / maxDiff
						/ std::abs(temperatureSpan);
	}

	float getZROChange() final { return activeZROChange; }

	void printTemperatureCalibrationState() final {
		temperatureCalibrator.printStatus();
	}

	void printDebugTemperatureCalibrationState() final {
		temperatureCalibrator.printSamples();
	}

	void resetTemperatureCalibrationState() final {
		temperatureCalibrator.stopLearning(false);
	}

	void saveTemperatureCalibration() final {
		temperatureCalibrator.saveConfig();
	}

	void startTemperatureCalibration() final {
		temperatureCalibrator.startLearning(false);
	}

	void stopTemperatureCalibration() final {
		temperatureCalibrator.stopLearning(true);
	}

	void setBackgroundTemperatureCalibration(bool enabled) final {
		temperatureCalibrator.setBackgroundLearning(enabled);
	}

	void clearTemperatureCalibration() final {
		temperatureCalibrator.clear();
	}

private:
	void getGyroOffset(float gyroOffset[3]) {
		gyroOffset[0] = activeCalibration.G_off1[0];
		gyroOffset[1] = activeCalibration.G_off1[1];
		gyroOffset[2] = activeCalibration.G_off1[2];

#if ENABLE_GYRO_TEMP_CURVE
		if (std::isfinite(currentTemperature)
			&& temperatureCalibrator.approximateOffset(
				currentTemperature,
				gyroOffset
			)) {
			return;
		}
#endif

		if (activeCalibration.gyroPointsCalibrated < 2
			|| !std::isfinite(currentTemperature)) {
			return;
		}
		const float temperatureSpan
			= activeCalibration.gyroMeasurementTemperature2
			- activeCalibration.gyroMeasurementTemperature1;
		if (std::abs(temperatureSpan) <= MinimumTemperatureSpan) {
			return;
		}
		const float interpolation = std::clamp(
			(currentTemperature - activeCalibration.gyroMeasurementTemperature1)
				/ temperatureSpan,
			0.0f,
			1.0f
		);
		for (size_t i = 0; i < 3; i++) {
			gyroOffset[i]
				+= interpolation
				 * (activeCalibration.G_off2[i] - activeCalibration.G_off1[i]);
		}
	}

	static void setIdentity(float matrix[3][3]) {
		for (size_t row = 0; row < 3; row++) {
			for (size_t column = 0; column < 3; column++) {
				matrix[row][column] = row == column ? 1.0f : 0.0f;
			}
		}
	}

	static void sanitizeCalibration(
		Configuration::RuntimeCalibrationSensorConfig& value
	) {
		if (!isValidAxisRemap(value.imuAxisRemap)) {
			value.imuAxisRemap = AXIS_REMAP_DEFAULT_SENSOR;
		}
		if (!isValidAxisRemap(value.magAxisRemap)) {
			value.magAxisRemap = AXIS_REMAP_DEFAULT_SENSOR;
		}

		bool validMagCalibration = value.magCalibrated;
		for (size_t axis = 0; axis < 3; axis++) {
			validMagCalibration
				= validMagCalibration && std::isfinite(value.M_B[axis]);
			for (size_t column = 0; column < 3; column++) {
				validMagCalibration = validMagCalibration
					&& std::isfinite(value.M_Ainv[axis][column]);
			}
		}
		if (!validMagCalibration) {
			value.magCalibrated = false;
			std::fill(value.M_B, value.M_B + 3, 0.0f);
			setIdentity(value.M_Ainv);
		}
		for (size_t axis = 0; axis < 3; axis++) {
			if (!std::isfinite(value.A_scale[axis])
				|| value.A_scale[axis] < 0.5f || value.A_scale[axis] > 1.5f) {
				value.A_scale[axis] = 1.0f;
			}
		}
	}

	void refreshActiveCalibration() {
		sanitizeCalibration(calibration);
		activeCalibration = calibration;
		if (toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			return;
		}

		activeCalibration.gyroPointsCalibrated = 0;
		for (size_t i = 0; i < 3; i++) {
			activeCalibration.G_off1[i] = 0;
			activeCalibration.G_off2[i] = 0;
			activeCalibration.accelCalibrated[i] = false;
			activeCalibration.A_off[i] = 0;
			activeCalibration.A_scale[i] = 1;
			activeCalibration.M_B[i] = 0;
		}
		activeCalibration.magCalibrated = false;
		setIdentity(activeCalibration.M_Ainv);
	}

	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
		MAG,
	};

	void computeNextCalibrationStep() {
		if (!calibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (calibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		} else if (!accelBiasCalibrationStep.allSidesCalibrated()) {
			nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			currentStep = &accelBiasCalibrationStep;
			logger.info(
				"[CAL] Place an uncalibrated tracker face level and keep it still"
			);
		} else if (magnetometerAvailable && !calibration.magCalibrated) {
			nextCalibrationStep = CalibrationStepEnum::MAG;
			currentStep = &magCalibrationStep;
			logger.info(
				"[CAL] Rotate the tracker smoothly through all axes for at least "
				"30 seconds"
			);
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		const CalibrationStepEnum completedStep = nextCalibrationStep;
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				computeNextCalibrationStep();

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				computeNextCalibrationStep();

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allSidesCalibrated()) {
					skippedAStep = true;
				}
				break;
			case CalibrationStepEnum::MAG:
				computeNextCalibrationStep();
				if (print) {
					printCalibration(CalibrationPrintFlags::MAG);
				}
				break;
		}

		// Calibration steps update the persistent copy. Make the new values effective
		// immediately instead of waiting for the next reboot.
		refreshActiveCalibration();
		calculateZROChange();
		if (completedStep == CalibrationStepEnum::SAMPLING_RATE) {
			Base::recalcFusion();
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
		if (!startupSequenceComplete && hasRequiredCalibration()
			&& completedStep != CalibrationStepEnum::SAMPLING_RATE
			&& completedStep != CalibrationStepEnum::MOTIONLESS) {
			startupSequenceComplete = true;
			logger.info("[CAL] Startup calibration complete; tracking enabled");
		}
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		calibration.data.runtimeCalibration = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
		MAG = 16,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS
		| CalibrationPrintFlags::MAG;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature1,
					calibration.G_off1[0],
					calibration.G_off1[1],
					calibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature2,
					calibration.G_off2[0],
					calibration.G_off2[1],
					calibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::MAG)) {
			if (calibration.magCalibrated) {
				logger.info(
					"Calibrated magnetometer bias: %f %f %f",
					calibration.M_B[0],
					calibration.M_B[1],
					calibration.M_B[2]
				);
			} else {
				logger.info("Magnetometer not calibrated");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{calibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		calibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{calibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		calibration,
		static_cast<float>(Consts::AScale)
	};
	MagCalibrationStep<RawSensorT> magCalibrationStep{calibration};
	NullCalibrationStep<RawSensorT> nullCalibrationStep{calibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;

	SlimeVR::Configuration::RuntimeCalibrationSensorConfig calibration{
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},
		.A_scale = {1.0, 1.0, 1.0},
		.accelSideCalibrated = {false, false, false, false, false, false},
		.accelSideAverage = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},

		.imuAxisRemap = AXIS_REMAP_DEFAULT_SENSOR,
		.magAxisRemap = AXIS_REMAP_DEFAULT_SENSOR,

		.magCalibrated = false,
		.M_B = {0.0, 0.0, 0.0},
		.M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
	};

	float activeZROChange = 0;
	static constexpr float MinimumTemperatureSpan = 0.1f;
	float currentTemperature = std::numeric_limits<float>::quiet_NaN();
	GyroTemperatureCalibrator temperatureCalibrator;
	bool magnetometerAvailable = false;
	bool startupSequenceComplete = false;

	Configuration::RuntimeCalibrationSensorConfig activeCalibration = calibration;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
