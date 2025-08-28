#include "SensorManager.h"
#include "config.h"
#include <Arduino.h>
#include <cstring>

// Static member definitions
bool SensorManager::sensorsInitialized = false;
SensorManager::CalibrationData SensorManager::calibration = {20.93f, 0.0f, 0.0f, 1.0f, false};
KalmanFilter SensorManager::pressureFilter(0.0f, KALMAN_Q_DEFAULT, KALMAN_R_DEFAULT);
KalmanFilter SensorManager::o2Filter(20.93f, KALMAN_Q_DEFAULT, KALMAN_R_DEFAULT);
KalmanFilter SensorManager::tempFilter(25.0f, KALMAN_Q_DEFAULT, KALMAN_R_DEFAULT);
MovingAverage<float, MOVING_AVG_WINDOW> SensorManager::co2Filter;
uint8_t SensorManager::sensorErrors[8] = {0};
unsigned long SensorManager::lastErrorTime[8] = {0};

// Pre-calculated constants for optimization
const float SensorManager::VENTURI_AREA = VENTURI_AREA_M2;
const float SensorManager::VENTURI_COEFFICIENT = VENTURI_COEFFICIENT;
const float SensorManager::AIR_DENSITY_STD = AIR_DENSITY_STP;
const float SensorManager::GAS_CONSTANT = GAS_CONSTANT_R;

// Sensor instances (conditional compilation based on config)
#ifdef USE_MAX30102
MAX30102 SensorManager::heartSensor;
#endif

#ifdef USE_MAX30105
MAX30105 SensorManager::bioSensor;
#endif

#ifdef USE_VL53L0X
VL53L0X SensorManager::tofSensor;
#endif

#ifdef USE_MLX90614
Adafruit_MLX90614 SensorManager::thermalSensor;
#endif

#ifdef USE_BMP280
Adafruit_BMP280 SensorManager::envSensor;
#endif

#ifdef USE_QMI8658
QMI8658 SensorManager::imuSensor;
#endif

bool SensorManager::begin() {
    Serial.println("Initializing sensors...");
    
    // Load calibration data from EEPROM/preferences
    loadCalibration();
    
    // Initialize individual sensors
    if (!initializeIndividualSensors()) {
        Serial.println("ERROR: Failed to initialize one or more sensors");
        return false;
    }
    
    // Perform self-test
    if (!performSelfTest()) {
        Serial.println("WARNING: Some sensors failed self-test");
        // Continue anyway - some sensors may be optional
    }
    
    sensorsInitialized = true;
    Serial.println("Sensor initialization complete");
    return true;
}

bool SensorManager::readSensors(SensorData& output) {
    if (!sensorsInitialized) {
        Serial.println("ERROR: Sensors not initialized");
        return false;
    }
    
    // Clear output structure
    memset(&output, 0, sizeof(SensorData));
    output.timestamp = millis();
    
    bool success = true;
    float ambientPressure = 101325.0f; // Default atmospheric pressure
    
    #ifdef USE_MAX30102
    // Read heart rate and SpO2
    if (heartSensor.check()) {
        float rawHR = 0.0f; // Placeholder - implement actual HR calculation
        float rawSpO2 = 95.0f; // Placeholder
        
        // Apply filtering (these would be stored in output structure)
        // For now, just demonstrate the concept
        output.sensorValid[0] = true;
    } else {
        output.sensorValid[0] = false;
        sensorErrors[0]++;
        success = false;
    }
    #endif
    
    #ifdef USE_MAX30105
    // Read NIRS reflectance data
    if (bioSensor.available()) {
        output.irReflectance = bioSensor.getIR() / 65536.0f;  // Normalize
        output.redReflectance = bioSensor.getRed() / 65536.0f;
        output.sensorValid[1] = true;
    } else {
        output.sensorValid[1] = false;
        sensorErrors[1]++;
        success = false;
    }
    #endif
    
    #ifdef USE_VL53L0X
    // Read distance/tissue thickness
    uint16_t distance = tofSensor.readRangeSingleMillimeters();
    if (distance < 8000) {  // Valid reading
        output.distance = distance;
        output.tissueThickness = distance * 0.1f;  // Convert to tissue thickness estimate
        output.sensorValid[2] = true;
    } else {
        output.sensorValid[2] = false;
        sensorErrors[2]++;
        success = false;
    }
    #endif
    
    #ifdef USE_MLX90614
    // Read non-contact temperature
    float objTemp = thermalSensor.readObjectTempC();
    if (objTemp > 30.0f && objTemp < 45.0f) {  // Reasonable range
        output.ambientTemp = tempFilter.update(objTemp);
        output.sensorValid[3] = true;
    } else {
        output.sensorValid[3] = false;
        sensorErrors[3]++;
        success = false;
    }
    #endif
    
    #ifdef USE_BMP280
    // Read environmental conditions
    if (envSensor.takeForcedMeasurement()) {
        float rawPressure = envSensor.readPressure();
        float rawTemp = envSensor.readTemperature();
        
        output.ambientPressure = pressureFilter.update(rawPressure);
        // Use environmental temp if object temp not available
        if (!output.sensorValid[3]) {
            output.ambientTemp = tempFilter.update(rawTemp);
        }
        output.sensorValid[4] = true;
    } else {
        output.sensorValid[4] = false;
        sensorErrors[4]++;
        success = false;
    }
    #endif
    
    #ifdef USE_QMI8658
    // Read IMU data (acceleration, gyroscope, magnetometer if available)
    // This is a placeholder - actual implementation depends on QMI8658 library
    // For now, generate some dummy data to test the structure
    output.acceleration[0] = 0.1f;
    output.acceleration[1] = 0.1f;
    output.acceleration[2] = 9.8f;
    output.gyroscope[0] = 0.0f;
    output.gyroscope[1] = 0.0f;
    output.gyroscope[2] = 0.0f;
    output.sensorValid[5] = true;
    #endif
    
    // Calculate overall signal quality
    int validSensors = 0;
    int totalSensors = 0;
    for (int i = 0; i < 8; i++) {
        if (output.sensorValid[i]) validSensors++;
        totalSensors++;
    }
    output.signalQuality = totalSensors > 0 ? (float)validSensors / totalSensors : 0.0f;
    
    // Update sensor health tracking
    updateSensorHealth();
    
    return success;
}

float SensorManager::calculateFlowRate(float pressure, float temperature) {
    // Optimized Bernoulli equation implementation with pre-calculated constants
    float density = calculateDensity(temperature, ambientPressure);
    float deltaP = pressure; // Differential pressure
    
    // Apply simplified Bernoulli equation: Q = Cd * A * sqrt(2 * deltaP / density)
    float flowRate = VENTURI_COEFFICIENT * VENTURI_AREA * sqrt(2.0f * abs(deltaP) / density);
    
    // Convert from m³/s to L/min
    return flowRate * 60000.0f;
}

float SensorManager::calculateMassFlow(float volumetricFlow, float density) {
    // Simple mass flow calculation: dm/dt = ρ * dV/dt
    return volumetricFlow * density / 60.0f; // Convert L/min to kg/s
}

bool SensorManager::initializeIndividualSensors() {
    bool allSuccess = true;
    
    #ifdef USE_MAX30102
    if (!heartSensor.begin()) {
        Serial.println("Failed to initialize MAX30102");
        allSuccess = false;
    } else {
        heartSensor.setup(); // Use default settings
        Serial.println("MAX30102 initialized");
    }
    #endif
    
    #ifdef USE_MAX30105
    if (!bioSensor.begin()) {
        Serial.println("Failed to initialize MAX30105");
        allSuccess = false;
    } else {
        bioSensor.setup(); // Use default settings for NIRS
        Serial.println("MAX30105 initialized");
    }
    #endif
    
    #ifdef USE_VL53L0X
    if (!tofSensor.init()) {
        Serial.println("Failed to initialize VL53L0X");
        allSuccess = false;
    } else {
        tofSensor.setTimeout(500);
        tofSensor.setMeasurementTimingBudget(20000);
        Serial.println("VL53L0X initialized");
    }
    #endif
    
    #ifdef USE_MLX90614
    if (!thermalSensor.begin()) {
        Serial.println("Failed to initialize MLX90614");
        allSuccess = false;
    } else {
        Serial.println("MLX90614 initialized");
    }
    #endif
    
    #ifdef USE_BMP280
    if (!envSensor.begin()) {
        Serial.println("Failed to initialize BMP280");
        allSuccess = false;
    } else {
        envSensor.setSampling(Adafruit_BMP280::MODE_FORCED,
                             Adafruit_BMP280::SAMPLING_X2,
                             Adafruit_BMP280::SAMPLING_X16,
                             Adafruit_BMP280::FILTER_X16,
                             Adafruit_BMP280::STANDBY_MS_500);
        Serial.println("BMP280 initialized");
    }
    #endif
    
    #ifdef USE_QMI8658
    // Placeholder for QMI8658 initialization
    Serial.println("QMI8658 initialization - placeholder");
    #endif
    
    return allSuccess;
}

bool SensorManager::performSelfTest() {
    Serial.println("Performing sensor self-test...");
    
    // Implement basic functionality tests for each sensor
    // This is a simplified version - real implementation would be more thorough
    
    bool allPass = true;
    
    #ifdef USE_MAX30102
    // Test MAX30102 by checking if it responds
    if (heartSensor.check()) {
        Serial.println("MAX30102 self-test: PASS");
    } else {
        Serial.println("MAX30102 self-test: FAIL");
        allPass = false;
    }
    #endif
    
    // Similar tests for other sensors...
    
    return allPass;
}

void SensorManager::updateSensorHealth() {
    // Reset error counters periodically
    unsigned long currentTime = millis();
    for (int i = 0; i < 8; i++) {
        if (currentTime - lastErrorTime[i] > ERROR_RESET_INTERVAL) {
            sensorErrors[i] = 0;
            lastErrorTime[i] = currentTime;
        }
    }
}

float SensorManager::calculateDensity(float temperature, float pressure) {
    // Calculate air density using ideal gas law: ρ = P / (R_specific * T)
    // R_specific for air = 287 J/(kg·K)
    float tempKelvin = temperature + 273.15f;
    return pressure / (287.0f * tempKelvin);
}

float SensorManager::applyBernoulliEquation(float pressure1, float pressure2, float density) {
    // Simplified Bernoulli equation for flow calculation
    float deltaP = abs(pressure1 - pressure2);
    return sqrt(2.0f * deltaP / density);
}

bool SensorManager::loadCalibration() {
    // Placeholder for loading calibration from EEPROM/Preferences
    // For now, use default values
    Serial.println("Loading default calibration values");
    return true;
}

void SensorManager::shutdown() {
    Serial.println("Shutting down sensors...");
    
    #ifdef USE_MAX30102
    heartSensor.shutDown();
    #endif
    
    #ifdef USE_MAX30105
    bioSensor.shutDown();
    #endif
    
    sensorsInitialized = false;
    Serial.println("Sensors shutdown complete");
}

// Stub implementations for other methods
bool SensorManager::calibrateO2(float reference) {
    calibration.o2Reference = reference;
    Serial.printf("O2 calibration set to %.2f%%\n", reference);
    return true;
}

bool SensorManager::calibrateFlow(float referenceVolume) {
    Serial.printf("Flow calibration with %.1fL reference\n", referenceVolume);
    return true;
}

bool SensorManager::isSensorHealthy(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= 8) return false;
    return sensorErrors[sensorIndex] < MAX_SENSOR_ERRORS;
}

const char* SensorManager::getSensorErrorMessage(int sensorIndex) {
    // Return appropriate error message based on sensor index
    static const char* errorMessages[] = {
        "MAX30102 Error", "MAX30105 Error", "VL53L0X Error", 
        "MLX90614 Error", "BMP280 Error", "QMI8658 Error",
        "Sensor 6 Error", "Sensor 7 Error"
    };
    
    if (sensorIndex >= 0 && sensorIndex < 8) {
        return errorMessages[sensorIndex];
    }
    return "Unknown Sensor Error";
}