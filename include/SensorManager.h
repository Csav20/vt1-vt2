#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "utils/MovingAverage.h"
#include "utils/KalmanFilter.h"

// Sensor-specific includes
#ifdef USE_MAX30102
#include <MAX30102.h>
#endif

#ifdef USE_MAX30105
#include <MAX30105.h>
#endif

#ifdef USE_VL53L0X
#include <VL53L0X.h>
#endif

#ifdef USE_MLX90614
#include <Adafruit_MLX90614.h>
#endif

#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#endif

#ifdef USE_QMI8658
#include <QMI8658.h>
#endif

/**
 * @brief Manages all sensor operations with error detection and validation
 * 
 * Features:
 * - Comprehensive sensor initialization and error checking
 * - Advanced filtering (Kalman, Moving Average)
 * - Automatic sensor calibration and validation
 * - Sensor-specific error detection and recovery
 */
class SensorManager {
public:
    struct SensorData {
        // Primary measurements
        float pressure;         // Pa
        float o2Percent;        // %
        float co2Ppm;           // ppm
        float ambientTemp;      // °C
        float ambientPressure;  // Pa
        float humidity;         // %
        
        // Advanced measurements
        float irReflectance;    // Normalized 0-1
        float redReflectance;   // Normalized 0-1
        float tissueThickness;  // mm
        float distance;         // mm (ToF sensor)
        
        // Motion data
        float acceleration[3];  // m/s²
        float gyroscope[3];     // rad/s
        float magnetometer[3];  // µT
        
        // Quality indicators
        bool sensorValid[8];    // Individual sensor validity flags
        float signalQuality;    // Overall signal quality 0-1
        uint32_t timestamp;     // Measurement timestamp
    };

    struct CalibrationData {
        float o2Reference;      // Reference O2 percentage
        float pressureOffset;   // Pressure sensor offset
        float tempOffset;       // Temperature offset
        float flowCalibFactor;  // Flow calibration factor
        bool calibrated;        // Calibration status
    };

    // Core sensor operations
    static bool begin();
    static bool readSensors(SensorData& output);
    static void shutdown();
    
    // Calibration functions
    static bool calibrateO2(float reference = 20.93f);
    static bool calibrateFlow(float referenceVolume = 3.0f);
    static bool calibrateEnvironmental();
    static void saveCalibration();
    static bool loadCalibration();
    
    // Sensor health and diagnostics
    static bool performSelfTest();
    static bool isSensorHealthy(int sensorIndex);
    static const char* getSensorErrorMessage(int sensorIndex);
    static void resetSensor(int sensorIndex);
    
    // Advanced features
    static void setFilterParameters(float kalmanQ, float kalmanR);
    static void enableAdaptiveFiltering(bool enable);
    static float getSignalQuality();
    
    // Optimized flow calculation with pre-calculated constants
    static float calculateFlowRate(float pressure, float temperature);
    static float calculateMassFlow(float volumetricFlow, float density);
    
private:
    // Sensor instances
    #ifdef USE_MAX30102
    static MAX30102 heartSensor;
    #endif
    
    #ifdef USE_MAX30105
    static MAX30105 bioSensor;
    #endif
    
    #ifdef USE_VL53L0X
    static VL53L0X tofSensor;
    #endif
    
    #ifdef USE_MLX90614
    static Adafruit_MLX90614 thermalSensor;
    #endif
    
    #ifdef USE_BMP280
    static Adafruit_BMP280 envSensor;
    #endif
    
    #ifdef USE_QMI8658
    static QMI8658 imuSensor;
    #endif
    
    // Filters for each sensor
    static KalmanFilter pressureFilter;
    static KalmanFilter o2Filter;
    static KalmanFilter tempFilter;
    static MovingAverage<float, 10> co2Filter;
    
    // Pre-calculated constants for optimization
    static const float VENTURI_AREA;          // m²
    static const float VENTURI_COEFFICIENT;   // Dimensionless
    static const float AIR_DENSITY_STD;       // kg/m³ at STP
    static const float GAS_CONSTANT;          // J/(mol·K)
    
    // Calibration data
    static CalibrationData calibration;
    static bool sensorsInitialized;
    
    // Error tracking
    static uint8_t sensorErrors[8];
    static unsigned long lastErrorTime[8];
    static const unsigned long ERROR_RESET_INTERVAL = 30000; // 30 seconds
    
    // Helper functions
    static bool initializeIndividualSensors();
    static void updateSensorHealth();
    static float compensateForTemperature(float rawValue, float temperature);
    static float compensateForPressure(float rawValue, float pressure);
    static bool validateSensorReading(int sensorIndex, float value);
    
    // Optimized calculations (moved from inline to reduce calculation overhead)
    static float calculateDensity(float temperature, float pressure);
    static float applyBernoulliEquation(float pressure1, float pressure2, float density);
};