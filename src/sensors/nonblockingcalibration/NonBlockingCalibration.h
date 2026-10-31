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

#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>

#include "../../GlobalVars.h"
#include "../../configuration/Configuration.h"
#include "../../utils.h"
#include "../SensorFusionRestDetect.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"

namespace SlimeVR::Sensors::NonBlockingCalibration {

template <typename IMU, typename SensorRawT>
class NonBlockingCalibrator {
public:
	mutable SlimeVR::Logging::Logger logger{"DynamicCalibration"};

	NonBlockingCalibrator(
		SensorFusionRestDetect& fusion,
		float accelScale,
		IMU& imu,
		uint8_t sensorId
	)
		: fusion{fusion}
		, imu{imu}
		, accelScale{accelScale}
		, sensorId{sensorId} {}

	void setup(Configuration::NonBlockingSensorConfig& baseCalibrationConfig) {
		sensorConfig = baseCalibrationConfig;
		startupMillis = millis();
		// printCalibration() below already says where gyro bias stands, so the
		// first periodic report is a minute away rather than immediate.
		lastGyroBiasReportMillis = startupMillis;

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		computeNextCalibrationStep();

		printCalibration();
	}

	void tick() {
		if (fusion.getRestDetected()) {
			ledManager.on();
		} else {
			ledManager.off();
		}
		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		reportGyroBiasProgress();

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
			case CalibrationStep<SensorRawT>::TickResult::DONE:
				stepCalibrationForward();
				break;
			case CalibrationStep<SensorRawT>::TickResult::SKIP:
				stepCalibrationForward(false);
				break;
			case CalibrationStep<SensorRawT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void provideAccelSample(const SensorRawT accelSample[3]) {
		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const SensorRawT gyroSample[3]) {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideTempSample(float tempSample) {
		// Kept whether or not a step is running, so the progress report below
		// can say what the temperature is even while nothing is calibrating.
		lastTempSample = tempSample;
		haveTempSample = true;

		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	/// The live calibration, which is newer than the copy the sensor loaded at
	/// boot once any step has saved. Pair it with getCalibrationRevision().
	const SlimeVR::Configuration::NonBlockingSensorConfig& getCalibration() const {
		return sensorConfig;
	}

	/// Bumped every time calibration is saved, so a caller can tell when the
	/// values above have changed without comparing the whole struct.
	uint32_t getCalibrationRevision() const { return calibrationRevision; }

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
	};

	void computeNextCalibrationStep() {
		if (!sensorConfig.sensorTimestepsCalibrated) {
			nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;
			currentStep = &sampleRateCalibrationStep;
		} else if (!sensorConfig.motionlessCalibrated && HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (sensorConfig.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (save) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (save) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (sensorConfig.gyroPointsCalibrated == 1) {
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				}

				if (save) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (save) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration;
		calibration.type = SlimeVR::Configuration::SensorConfigType::NONBLOCKING;
		calibration.data.nonblocking = sensorConfig;
		configuration.setSensor(sensorId, calibration);
		configuration.save();

		calibrationRevision++;

		ledManager.blink(100);
	}

	// Says where gyro bias calibration has got to, every so often, so a run can
	// be read afterwards and answer the question the numbers alone cannot: did
	// this tracker ever collect a second point? Without one there is nothing to
	// interpolate between and the bias correction is stuck at whatever
	// temperature the first point was taken at.
	void reportGyroBiasProgress() {
		if (!haveTempSample || sensorConfig.gyroPointsCalibrated == 2) {
			return;
		}

		if (millis() - lastGyroBiasReportMillis < gyroBiasReportSeconds * 1e3) {
			return;
		}
		lastGyroBiasReportMillis = millis();

		if (sensorConfig.gyroPointsCalibrated == 0) {
			logger.info(
				"Gyro bias: no points yet, %.1fC now - needs a stretch at rest",
				lastTempSample
			);
			return;
		}

		float apart
			= std::abs(lastTempSample - sensorConfig.gyroMeasurementTemperature1);
		logger.info(
			"Gyro bias: 1 point at %.1fC, %.1fC now (%.1fC apart, needs %.1fC) - no "
			"temperature interpolation until a second point lands",
			sensorConfig.gyroMeasurementTemperature1,
			lastTempSample,
			apart,
			GyroBiasCalibrationStep<SensorRawT>::temperatureDifferenceNeeded()
		);
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (sensorConfig.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					sensorConfig.A_Ts,
					sensorConfig.G_Ts,
					sensorConfig.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (HasMotionlessCalib && any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (sensorConfig.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (sensorConfig.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					sensorConfig.gyroMeasurementTemperature1,
					sensorConfig.G_off1[0],
					sensorConfig.G_off1[1],
					sensorConfig.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (sensorConfig.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					sensorConfig.gyroMeasurementTemperature2,
					sensorConfig.G_off2[0],
					sensorConfig.G_off2[1],
					sensorConfig.G_off2[2]
				);
				logger.info(
					"Gyro bias: 2 points, %.1fC to %.1fC - bias is interpolated to "
					"the current temperature",
					sensorConfig.gyroMeasurementTemperature1,
					sensorConfig.gyroMeasurementTemperature2
				);
			} else if (sensorConfig.gyroPointsCalibrated == 1) {
				logger.info(
					"Gyro bias: 1 point only, at %.1fC - no interpolation, and a "
					"second point needs a rest at least %.1fC away",
					sensorConfig.gyroMeasurementTemperature1,
					GyroBiasCalibrationStep<SensorRawT>::temperatureDifferenceNeeded()
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					sensorConfig.A_off[0],
					sensorConfig.A_off[1],
					sensorConfig.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					sensorConfig.A_off[0],
					sensorConfig.A_off[1],
					sensorConfig.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}
	}

	static constexpr bool HasMotionlessCalib
		= requires(IMU& i) { typename IMU::MotionlessCalibrationData; };

	SensorFusionRestDetect& fusion;
	IMU& imu;
	float accelScale;
	uint8_t sensorId;

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis;

	SlimeVR::Configuration::NonBlockingSensorConfig sensorConfig;

	SampleRateCalibrationStep<SensorRawT> sampleRateCalibrationStep{sensorConfig};
	MotionlessCalibrationStep<IMU, SensorRawT> motionlessCalibrationStep{
		sensorConfig,
		imu
	};
	GyroBiasCalibrationStep<SensorRawT> gyroBiasCalibrationStep{sensorConfig};
	AccelBiasCalibrationStep<SensorRawT> accelBiasCalibrationStep{
		sensorConfig,
		accelScale
	};
	NullCalibrationStep<SensorRawT> nullCalibrationStep{sensorConfig};

	CalibrationStep<SensorRawT>* currentStep = &nullCalibrationStep;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;

	static constexpr float gyroBiasReportSeconds = 60;
	uint64_t lastGyroBiasReportMillis = 0;
	float lastTempSample = 0;
	bool haveTempSample = false;

	uint32_t calibrationRevision = 0;
};

}  // namespace SlimeVR::Sensors::NonBlockingCalibration
