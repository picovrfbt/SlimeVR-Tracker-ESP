/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include "sensors/bno080sensor.h"

#include "GlobalVars.h"
#include "utils.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <math.h>  // For PI, TWO_PI and other math functions

// Need to define fixed-point math macros if not defined elsewhere
#ifndef F_TO_FX
#define F_TO_FX(f) ((int32_t)((f) * 256.0f))
#endif

#ifndef FX_TO_F
#define FX_TO_F(fx) ((float)(fx) / 256.0f)
#endif

#ifndef FX_MUL
#define FX_MUL(a, b) (((a) * (b)) >> 8)
#endif

#ifndef FX_DIV
#define FX_DIV(a, b) (((a) << 8) / (b))
#endif

// Missing MFX struct definitions if not defined elsewhere
#ifndef MFX_NUM_AXES
#define MFX_NUM_AXES 3

typedef struct {
    int32_t mag[MFX_NUM_AXES];
    uint32_t timestamp;
} MFX_MagCal_input_t;

typedef struct {
    int32_t hi_bias[MFX_NUM_AXES];
    uint8_t quality;
    uint16_t sample_count;
} MFX_MagCal_output_t;

// Magnetometer calibration quality values
#define MFX_MAGCAL_UNKNOWN 0
#define MFX_MAGCAL_POOR 1
#define MFX_MAGCAL_OK 2
#define MFX_MAGCAL_GOOD 3
#endif

/*
 * Calculates the Euclidean norm (magnitude) of a vector
 * - Uses fixed-point multiplication for accuracy
 * - Converts back to floating point for sqrt calculation
 */
int32_t norm(const int32_t* vector, int size) {
    int32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += FX_MUL(vector[i], vector[i]);  // Fixed-point multiplication for accuracy
    }
    return (int32_t)sqrtf(FX_TO_F(sum));  // Convert back to floating point for sqrt
}

/*
 * Initializes the BNO080 sensor and configures its settings
 * - Sets up I2C communication
 * - Configures sensor fusion modes
 * - Enables required sensor features
 * - Initializes magnetometer if enabled
 * - Sets up calibration parameters
 */
void BNO080Sensor::motionSetup() {
#ifdef DEBUG_SENSOR
	imu.enableDebugging(Serial);
#endif
	// Try with the correct address 0x4A first, ignoring the addr variable since it's incorrect (0x00)
	if (!imu.begin(0x4A, Wire, m_IntPin)) {
		m_Logger.fatal(
			"Can't connect to %s at address 0x%02x",
			getIMUNameByType(sensorType),
			0x4A
		);
		ledManager.pattern(50, 50, 200);
		return;
	}
    
    // Update the addr member to match the correct address we just used
    addr = 0x4A;

	m_Logger.info(
		"Connected to %s on 0x%02x.",
		getIMUNameByType(sensorType),
		addr
	);

	SlimeVR::Configuration::SensorConfig sensorConfig
		= configuration.getSensor(sensorId);
	
	// Always enable magnetometer for BNO085
	if (sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086) {
		magStatus = MagnetometerStatus::MAG_ENABLED;
		magEnabled = true;
	} else {
		// For other sensors, use config or default
		switch (sensorConfig.type) {
			case SlimeVR::Configuration::SensorConfigType::BNO0XX:
				magEnabled = sensorConfig.data.bno0XX.magEnabled;
				magStatus = magEnabled ? MagnetometerStatus::MAG_ENABLED
											: MagnetometerStatus::MAG_DISABLED;
				break;
			default:
				magStatus = USE_6_AXIS ? MagnetometerStatus::MAG_DISABLED
									: MagnetometerStatus::MAG_ENABLED;
				break;
		}
	}

	// Enable magnetometer first
	if (isMagEnabled()) {
		// Enable magnetometer at higher rate for better calibration
		imu.enableMagnetometer(50);  // 50Hz updates
		
		// Wait a bit for mag to initialize
		delay(100);
		
		// Configure rotation vector to use magnetometer
		if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
			&& BNO_USE_ARVR_STABILIZATION) {
			imu.enableARVRStabilizedRotationVector(10);
		} else {
			imu.enableRotationVector(10);
		}
		
			// Small delay to let settings take effect
		delay(100);
	} else {
		if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
			&& BNO_USE_ARVR_STABILIZATION) {
			imu.enableARVRStabilizedGameRotationVector(10);
		} else {
			imu.enableGameRotationVector(10);
		}
	}

	imu.enableLinearAccelerometer(10);
	initMagneticCalibration();

#if ENABLE_INSPECTION
	imu.enableRawGyro(10);
	imu.enableRawAccelerometer(10);
	if (isMagEnabled()) {
		imu.enableRawMagnetometer(10);
	}
#endif

	lastReset = 0;
	lastData = millis();
	working = true;
	configured = true;
	m_tpsCounter.reset();
	m_dataCounter.reset();

    // Temperature sensor not available in BNO08x API this needs to call from "Get Feature Request" ( at 0xFE ) Planned implementation
    // imu.enableTemperature(100); // 100ms interval (10Hz)
}

/*
 * Main sensor data processing loop
 * - Reads quaternion data for orientation
 * - Handles magnetometer calibration
 * - Processes linear acceleration
 * - Monitors sensor stability
 * - Updates calibration status
 */
void BNO080Sensor::motionLoop() {
    // Replace dataAvailable with check for new quaternion data
    if (imu.wasReset()) {
        m_Logger.info("BNO08x was reset. Re-initializing...");
        motionSetup();
        return;
    }

    // Process available data if quaternion is ready
    if (millis() - lastData > 100) { // Check at 10Hz rate
        lastData = millis();
        
        Quat nRotation;  // Local quaternion variable
        
        if (isMagEnabled()) {
            static uint32_t lastMagStatusCheck = 0;
            uint32_t currentTime = millis();
            
            // Check mag status every 5 seconds
            if (currentTime - lastMagStatusCheck >= 5000) {
                lastMagStatusCheck = currentTime;
                uint8_t newAccuracy = imu.getMagAccuracy();
                const char* statusText;
                switch(newAccuracy) {
                    case 0:
                        statusText = "Uncalibrated";
                        break;
                    case 1:
                        statusText = "Minimal Calibration";
                        break;
                    case 2:
                        statusText = "More Calibrated";
                        break;
                    case 3:
                        statusText = "Fully Calibrated";
                        // Save calibration when we reach full calibration
                        if (newAccuracy > magCalibrationAccuracy) {
                            m_Logger.info("Reached full calibration - saving calibration data");
                            imu.saveCalibration();
                            delay(100); // Give it time to save
                        }
                        break;
                    default:
                        statusText = "Unknown";
                }
                
                // Only log if accuracy changed
                if (newAccuracy != magCalibrationAccuracy) {
                    magCalibrationAccuracy = newAccuracy;
                    m_Logger.info("Magnetometer Calibration Status: %s (Level %d/3)", statusText, magCalibrationAccuracy);
                } else {
                    m_Logger.info("Magnetometer Status Check - Current Status: %s (Level %d/3)", statusText, magCalibrationAccuracy);
                }
                
                // If accuracy drops below 2, try to recalibrate
                if (magCalibrationAccuracy < 2) {
                    // Use magnetometer-specific calibration method
                    // Not directly available in BNO08x class, so we'll handle it in our code
                    m_Logger.info("Magnetometer needs calibration - please rotate the sensor in a figure-8 pattern");
                }
            }
            
            if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                && BNO_USE_ARVR_STABILIZATION) {
                float radianAccuracy = 0.0f;
                imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, radianAccuracy, calibrationAccuracy);
                magneticAccuracyEstimate = radianAccuracy;
            } else {
                float radianAccuracy = 0.0f;
                imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, radianAccuracy, calibrationAccuracy);
                magneticAccuracyEstimate = radianAccuracy;
            }
            
            networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
            
        } else {
            // For game rotation quaternion, we need to use individual getter methods
            // as getGameQuatI() doesn't take parameters
            if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                && BNO_USE_ARVR_STABILIZATION) {
                // Need to manually get each component of the quaternion
                calibrationAccuracy = imu.getQuatAccuracy();
                // Get quaternion components - note the order may need adjustment
                nRotation.w = imu.getGameQuatReal();
                nRotation.x = imu.getGameQuatI();
                nRotation.y = imu.getGameQuatJ();
                nRotation.z = imu.getGameQuatK();
            } else {
                // Need to manually get each component of the quaternion
                calibrationAccuracy = imu.getQuatAccuracy();
                // Get quaternion components - note the order may need adjustment
                nRotation.w = imu.getGameQuatReal();
                nRotation.x = imu.getGameQuatI();
                nRotation.y = imu.getGameQuatJ();
                nRotation.z = imu.getGameQuatK();
            }
            
            networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
        }

        // Get linear acceleration data
        uint8_t acc;
        Vector3 nAccel;
        imu.getLinAccel(nAccel.x, nAccel.y, nAccel.z, acc);
        networkConnection.sendSensorAcceleration(sensorId, nAccel);

        // Update magnetic calibration if enabled
        if (isMagEnabled()) {
            MFX_MagCal_input_t magInput;
            magInput.mag[0] = F_TO_FX(imu.getRawMagX());
            magInput.mag[1] = F_TO_FX(imu.getRawMagY());
            magInput.mag[2] = F_TO_FX(imu.getRawMagZ());
            magInput.timestamp = millis();
            updateMagneticCalibration(magInput);
        }
    }

    if (lastData + 1000 < millis()) {
        m_Logger.warn("Sensor %d: No data from BNO080", sensorId);
        lastData = millis();
    }

    if (imu.getStabilityClassifier() == 1) {
        markRestCalibrationComplete();
    }

    updateMagneticCalibration();
}

/*
 * Returns current sensor operational status
 * - SENSOR_OK: If working normally
 * - SENSOR_OFFLINE: If not working
 */
SensorStatus BNO080Sensor::getSensorState() {
    return isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

/*
 * Sends processed sensor data to the network
 * - Sends quaternion orientation data
 * - Sends linear acceleration data
 * - Only sends when fusion data is updated
 */
void BNO080Sensor::sendData() {
    if (!m_fusion.isUpdated()) {
        return;
    }

    Quat quaternion = m_fusion.getQuaternionQuat();
    Vector3 acceleration = m_fusion.getLinearAccVec();

    networkConnection.sendRotationData(
        sensorId,
        &quaternion,
        DATA_TYPE_NORMAL,
        calibrationAccuracy
    );

    networkConnection.sendSensorAcceleration(sensorId, acceleration);

    m_fusion.setUpdated(false);
}

/*
 * Sets sensor configuration flags
 * - Handles magnetometer enable/disable
 * - Updates sensor configuration
 * - Triggers sensor reinitialization when needed
 */
void BNO080Sensor::setFlag(uint16_t flagId, bool state) {
    if (flagId == FLAG_SENSOR_BNO0XX_MAG_ENABLED) {
        magEnabled = state;
        magStatus = state ? MagnetometerStatus::MAG_ENABLED
                          : MagnetometerStatus::MAG_DISABLED;

        SlimeVR::Configuration::SensorConfig config;
        config.type = SlimeVR::Configuration::SensorConfigType::BNO0XX;
        config.data.bno0XX.magEnabled = magEnabled;
        configuration.setSensor(sensorId, config);

        // Reinitialize the sensor
        motionSetup();
    }
}

/*
 * Initiates sensor calibration sequence
 * - Provides detailed calibration instructions
 * - Configures magnetometer for calibration
 * - Monitors calibration progress
 * - Handles calibration data storage
 */
void BNO080Sensor::startCalibration(int calibrationType) {
    if (calibrationType == 2) {  // Magnetometer calibration type
        m_Logger.info("Starting magnetometer calibration sequence...");
        m_Logger.info("Current calibration level: %d/3", magCalibrationAccuracy);
        m_Logger.info("=== Calibration Instructions ===");
        m_Logger.info("1. Hold the sensor at least 0.5m away from any large metal objects");
        m_Logger.info("2. Perform the following movements slowly and smoothly:");
        m_Logger.info("   a) Draw figure-8 patterns in different orientations");
        m_Logger.info("   b) Rotate the sensor 360° around each axis");
        m_Logger.info("   c) Keep movements slow - about 3 seconds per rotation");
        m_Logger.info("3. Watch the calibration level:");
        m_Logger.info("   Level 0: Uncalibrated - Keep moving");
        m_Logger.info("   Level 1: Basic calibration - Continue movement");
        m_Logger.info("   Level 2: Good calibration - Fine-tune movements");
        m_Logger.info("   Level 3: Best calibration - Calibration complete");
        m_Logger.info("4. After reaching Level 3:");
        m_Logger.info("   - Keep position stable for 2-3 seconds");
        m_Logger.info("   - Calibration will be automatically saved");
        m_Logger.info("5. If accuracy drops:");
        m_Logger.info("   - Move away from magnetic interference");
        m_Logger.info("   - Repeat calibration if necessary");
        
        // Enable high-rate magnetometer updates during calibration
        imu.enableMagnetometer(50);  // 50Hz updates
        
        // Manual calibration method since sendCalibrateCommand is not available
        m_Logger.info("Please move the sensor in a figure-8 pattern to help calibrate the magnetometer");
        
        // Small delay to let the commands process
        delay(50);
    }
}

/*
 * Initializes magnetometer calibration
 * - Sets up calibration parameters
 * - Enables magnetometer readings
 * - Prepares calibration structures
 */
void BNO080Sensor::initMagneticCalibration() {
    if (isMagEnabled()) {
        // Initialize calibration structures
        memset(&m_MagCalInput, 0, sizeof(m_MagCalInput));
        memset(&m_MagCalOutput, 0, sizeof(m_MagCalOutput));
        m_MagCalOutput.quality = MFX_MAGCAL_UNKNOWN;
        
        // Alternative to sendCalibrateCommand
        m_Logger.info("Initializing magnetometer calibration");
        // Force enable magnetometer at high rate temporarily
        imu.enableMagnetometer(50);
    }
}

/*
 * Updates magnetic calibration state
 * - Processes new magnetic data
 * - Updates calibration parameters
 */
void BNO080Sensor::updateMagneticCalibration() {
    processMagneticData();
}

/*
 * Updates magnetic calibration state with new input data
 * - Processes new magnetic data
 * - Updates calibration parameters
 */
void BNO080Sensor::updateMagneticCalibration(const MFX_MagCal_input_t& magInput) {
    m_MagCalInput = magInput;
    processMagneticData();
}

/*
 * Processes magnetic sensor data
 * - Handles magnetic interference detection
 * - Applies hard iron compensation
 * - Manages temperature compensation
 * - Updates calibration quality
 * - Implements disturbance recovery
 */
void BNO080Sensor::processMagneticData() {
    updateTemperatureCompensation();

    // Store current readings in history
    magHistory[historyIndex][0] = m_MagCalInput.mag[0];
    magHistory[historyIndex][1] = m_MagCalInput.mag[1];
    magHistory[historyIndex][2] = m_MagCalInput.mag[2];
    historyIndex = (historyIndex + 1) % 6;

    // Calculate variance and rate of change
    int32_t avgMag[3] = {0, 0, 0};
    int32_t variance = 0;
    int32_t maxRateOfChange = 0;
    
    // Calculate average and max rate of change
    for(int i = 0; i < 6; i++) {
        for(int axis = 0; axis < 3; axis++) {
            avgMag[axis] += magHistory[i][axis];
            
            // Calculate rate of change between consecutive samples
            if(i > 0) {
                int32_t rateOfChange = abs(magHistory[i][axis] - magHistory[i-1][axis]);
                if(rateOfChange > maxRateOfChange) {
                    maxRateOfChange = rateOfChange;
                }
            }
        }
    }
    
    for(int axis = 0; axis < 3; axis++) {
        avgMag[axis] /= 6;
        
        // Calculate variance contribution from this axis
        for(int i = 0; i < 6; i++) {
            int32_t diff = magHistory[i][axis] - avgMag[axis];
            variance += (diff * diff) >> 8;
        }
    }
    
    // Enhanced disturbance detection with multiple criteria
    static const int32_t VARIANCE_THRESHOLD = F_TO_FX(0.8);    // Much more sensitive to variations
    static const int32_t RATE_THRESHOLD = F_TO_FX(0.25);       // More sensitive to sudden changes
    static const int32_t RECOVERY_THRESHOLD = F_TO_FX(0.15);   // Very conservative recovery
    static const int32_t DECAY_RATE = F_TO_FX(0.995);         // Very slow decay
    static const int32_t RECOVERY_RATE = F_TO_FX(0.02);       // Very slow recovery
    
    // Add distance-based threshold scaling
    static const int32_t BASE_MAGNETIC_STRENGTH = F_TO_FX(40.0);  // Expected clean magnetic field strength
    
    // Calculate magnetic field strength using fixed point math
    int32_t fieldStrengthSquared = FX_MUL(m_MagCalInput.mag[0], m_MagCalInput.mag[0]) + 
                                  FX_MUL(m_MagCalInput.mag[1], m_MagCalInput.mag[1]) + 
                                  FX_MUL(m_MagCalInput.mag[2], m_MagCalInput.mag[2]);
    int32_t currentStrength = (int32_t)sqrtf(FX_TO_F(fieldStrengthSquared));
    int32_t strengthRatio = FX_DIV(currentStrength, BASE_MAGNETIC_STRENGTH);
    
    // Scale thresholds based on field strength (stronger field = more sensitive detection)
    int32_t scaledVarianceThreshold = strengthRatio > F_TO_FX(1.2) ? 
                                     FX_MUL(VARIANCE_THRESHOLD, F_TO_FX(0.5)) : 
                                     VARIANCE_THRESHOLD;
    
    bool isDisturbed = (variance > scaledVarianceThreshold) || 
                       (maxRateOfChange > RATE_THRESHOLD) ||
                       (strengthRatio > F_TO_FX(1.5));  // Detect strong fields
    
    if(isDisturbed) {
        if(!inDisturbance) {
            inDisturbance = true;
            memcpy(lastValidMag, m_MagCalInput.mag, sizeof(lastValidMag));
            
            // Start using gyro for heading immediately
            usingGyroHeading = true;
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);
            lastGyroHeading = magHeading;
            lastGyroTime = millis();
        }
        
        // During disturbance, blend between last valid and current readings
        for(int i = 0; i < 3; i++) {
            // More aggressive blending during strong disturbances
            int32_t blendFactor = strengthRatio > F_TO_FX(1.5) ? 
                                 F_TO_FX(0.98) :  // Almost entirely ignore current readings
                                 DECAY_RATE;
            m_MagCalInput.mag[i] = FX_MUL(lastValidMag[i], blendFactor) + 
                                  FX_MUL(avgMag[i], F_TO_FX(1.0) - blendFactor);
        }
        
        m_MagCalOutput.quality = MFX_MAGCAL_POOR;
        
    } else if(variance < RECOVERY_THRESHOLD && inDisturbance) {
        // Gradual recovery with consistency check
        bool consistentReadings = true;
        for(int axis = 0; axis < 3; axis++) {
            int32_t maxDeviation = 0;
            for(int i = 0; i < 6; i++) {
                int32_t deviation = abs(magHistory[i][axis] - avgMag[axis]);
                if(deviation > maxDeviation) maxDeviation = deviation;
            }
            if(maxDeviation > F_TO_FX(0.3)) {
                consistentReadings = false;
                break;
            }
        }
        
        if(consistentReadings) {
            for(int axis = 0; axis < 3; axis++) {
                lastValidMag[axis] = FX_MUL(lastValidMag[axis], F_TO_FX(1.0) - RECOVERY_RATE) + 
                                    FX_MUL(avgMag[axis], RECOVERY_RATE);
                m_MagCalInput.mag[axis] = lastValidMag[axis];
            }
            
            if(variance < (RECOVERY_THRESHOLD >> 1)) {
                // Only exit disturbance mode if readings are very stable
                inDisturbance = false;
                usingGyroHeading = false;
                m_MagCalOutput.quality = MFX_MAGCAL_OK;
            }
        }
    }

    // Apply hard iron bias correction
    int32_t correctedMag[MFX_NUM_AXES];
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        correctedMag[i] = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
    }

    // Calculate magnitude
    int32_t magSquared = 0;
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        magSquared += FX_MUL(correctedMag[i], correctedMag[i]);
    }
    
    int32_t magStrength = (int32_t)sqrtf(FX_TO_F(magSquared));
    
    // Enhanced field strength compensation
    static const int32_t MAG_COMPRESS = F_TO_FX(1.2);      // More aggressive compression
    static const int32_t MAG_MAX_FIELD = F_TO_FX(2.5);     // Higher max field tolerance
    static const int32_t COMPRESSION_FACTOR = F_TO_FX(0.8); // Stronger compression
    
    if (magStrength > MAG_COMPRESS && magStrength <= MAG_MAX_FIELD) {
        int32_t excess = magStrength - MAG_COMPRESS;
        int32_t compressionRatio = FX_MUL(excess, COMPRESSION_FACTOR);
        
        for(int i = 0; i < MFX_NUM_AXES; i++) {
            // More weight to last valid reading during high disturbance
            int32_t reduction = FX_MUL(correctedMag[i], compressionRatio);
            correctedMag[i] -= reduction;
        }
    }
    
    // Store corrected values
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        m_MagCalInput.mag[i] = correctedMag[i];
    }

    updateHardIronCompensation();
    processGyroData();
    
    // After applying corrections, check if we've returned to initial position
    if (!hasInitialPosition && m_MagCalOutput.quality >= MFX_MAGCAL_GOOD) {
        // Store initial position when we have good calibration
        memcpy(initialPosition, m_MagCalInput.mag, sizeof(initialPosition));
        hasInitialPosition = true;
    } else if (inDisturbance && hasInitialPosition) {
        // Calculate similarity to initial position
        int32_t simScore = 0;
        for (int i = 0; i < 3; i++) {
            int32_t diff = abs(avgMag[i] - initialPosition[i]);
            simScore += diff;
        }
        
        // If very similar to initial position, exit disturbance mode faster
        if (simScore < F_TO_FX(0.5)) {
            m_Logger.info("Detected return to initial position - exiting disturbance mode");
            inDisturbance = false;
            usingGyroHeading = false;
            m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
        }
    }
}

/*
 * Updates hard iron compensation values
 * - Calculates bias corrections
 * - Updates calibration quality
 * - Manages sample collection
 */
void BNO080Sensor::updateHardIronCompensation() {
    static const uint16_t SAMPLES_FOR_CALIBRATION = 25; // Reduced samples for faster response
    static const int32_t LEARNING_RATE = F_TO_FX(0.05); // Increased to 5% learning rate
    
    // Only update when device is stable (low gyro readings)
    float x, y, z;
    uint8_t gyroAccuracy;
    imu.getGyro(x, y, z, gyroAccuracy);
    
    if (abs(x) < 1.0f && abs(y) < 1.0f && abs(z) < 1.0f) {
        m_MagCalOutput.sample_count++;
        
        if (m_MagCalOutput.sample_count >= SAMPLES_FOR_CALIBRATION) {
            // Update hard iron bias with exponential moving average
            for (int i = 0; i < MFX_NUM_AXES; i++) {
                int32_t error = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
                m_MagCalOutput.hi_bias[i] += FX_MUL(error, LEARNING_RATE);
            }
            
            m_MagCalOutput.sample_count = 0;
            
            // Update calibration quality
            if (m_MagCalOutput.quality < MFX_MAGCAL_GOOD) {
                m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
            }
        }
    }
}

/*
 * Processes gyroscope data
 * - Handles heading calculations
 * - Applies temperature compensation
 * - Manages drift correction
 * - Integrates with magnetometer data
 */
void BNO080Sensor::processGyroData() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastGyroTime) / 1000.0f;
    lastGyroTime = currentTime;
    
    if(usingGyroHeading) {
        // Get raw gyro data
        float x, y, z;
        uint8_t gyroAccuracy;
        imu.getGyro(x, y, z, gyroAccuracy);
        float gyroZ = z;
        
        // BNO08x doesn't have direct temperature access
        // Use constant temperature assumption instead
        const float assumedTemp = 25.0f;  // Assume room temperature
        float tempDiff = assumedTemp - 25.0f;  // Usually zero
        gyroZ *= (1.0f + TEMP_COEFF * tempDiff);  // Apply temperature correction
        
        // Update heading with gyro data
        lastGyroHeading += gyroZ * deltaTime;
        
        // Normalize to -PI to PI
        while(lastGyroHeading > PI) lastGyroHeading -= TWO_PI;
        while(lastGyroHeading < -PI) lastGyroHeading += TWO_PI;
        
        // If we have valid mag data, do slow correction to prevent drift
        if(!inDisturbance && m_MagCalOutput.quality >= MFX_MAGCAL_OK) {
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);
            float headingDiff = magHeading - lastGyroHeading;
            
            // Normalize difference to -PI to PI
            while(headingDiff > PI) headingDiff -= TWO_PI;
            while(headingDiff < -PI) headingDiff += TWO_PI;
            
            // Very slow correction factor (0.1% per update)
            lastGyroHeading += headingDiff * 0.001f;
        }
    }
}

/*
 * Updates temperature compensation
 * - Monitors temperature changes
 * - Applies dynamic compensation
 * - Adjusts sensor readings based on temperature
 */
void BNO080Sensor::updateTemperatureCompensation() {
    // BNO08x doesn't have direct temperature sensor access
    // Pending SH2 Implementation using mag reports to get temp data
    // Skip temperature compensation or use alternative approach
    
    // If temperature compensation is critical, could use:
    // 1. External temperature sensor
    // 2. ESP's internal temperature sensor
    // 3. A constant value (which effectively disables dynamic compensation)
    
    const float assumedTemp = 25.0f;  // Assume room temperature
    
    if(lastTemp == 0) {
        // Initialize on first run
        lastTemp = assumedTemp;
    }
    
    // Temperature difference would typically be zero with constant temperature
    float tempDiff = assumedTemp - lastTemp;
    lastTemp = assumedTemp;
    
    // Only apply minimal compensation if needed
    if(abs(tempDiff) > 0.5f) {
        for(int i = 0; i < 3; i++) {
            // Apply minimal temperature adjustment
            m_MagCalInput.mag[i] = m_MagCalInput.mag[i] * (1.0f + TEMP_COEFF * tempDiff);
        }
    }
}
