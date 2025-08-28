#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_OxygenSensor.h"
#include "SCD30.h"
#include "Omron_D6FPH.h"
#include "Adafruit_BMP280.h"
#include "KalmanFilter.h"

// Sensor addresses
#ifndef ADDRESS_3
#define ADDRESS_3 0x73
#endif

#ifndef OXYGEN_COLLECT_NUMBER
#define OXYGEN_COLLECT_NUMBER 10
#endif

/**
 * Sensor Manager - Unified sensor reading and filtering
 */
class SensorManager {
public:
    struct SensorData {
        float o2Percent;         // Oxygen percentage
        float co2Ppm;           // CO2 in PPM
        float pressure;         // Differential pressure (Pa)
        float temperature;      // Temperature (°C)
        float ambientPressure;  // Ambient pressure (hPa)
        float humidity;         // Humidity (%)
        float flowRate;         // Flow rate (L/min)
        bool o2Valid;           // O2 sensor status
        bool co2Valid;          // CO2 sensor status
        bool pressureValid;     // Pressure sensor status
        bool ambientValid;      // Ambient sensor status
        unsigned long timestamp; // Measurement timestamp
    };
    
    struct CalibrationData {
        float o2Correction;     // O2 calibration factor
        float pressureOffset;   // Pressure offset
        float flowCalibration;  // Flow calibration factor
    };
    
private:
    static DFRobot_OxygenSensor oxygenSensor;
    static Omron_D6FPH pressureSensor;
    static Adafruit_BMP280 ambientSensor;
    static bool sensorsInitialized;
    static bool co2SensorAvailable;
    
    // Kalman filters for each sensor
    static UnifiedKalmanFilter o2Filter;
    static UnifiedKalmanFilter co2Filter;
    static UnifiedKalmanFilter pressureFilter;
    static UnifiedKalmanFilter temperatureFilter;
    
    static CalibrationData calibration;
    static unsigned long lastReading;
    static const unsigned long READING_INTERVAL = 50; // 20Hz
    
public:
    /**
     * Initialize all sensors
     * @return true if at least one sensor is available
     */
    static bool initialize();
    
    /**
     * Read all sensors and apply filtering
     * @param data Output sensor data
     * @return true if successful
     */
    static bool readSensors(SensorData& data);
    
    /**
     * Calibrate O2 sensor
     * @param referenceO2 Reference O2 percentage (default: 20.93%)
     * @return true if successful
     */
    static bool calibrateO2(float referenceO2 = 20.93);
    
    /**
     * Calibrate pressure sensor (zero offset)
     * @return true if successful
     */
    static bool calibratePressure();
    
    /**
     * Calibrate flow using syringe method
     * @param syringeVolume Volume of syringe in mL
     * @param injectionTime Time of injection in seconds
     * @return true if successful
     */
    static bool calibrateFlow(float syringeVolume, float injectionTime);
    
    /**
     * Get calibration data
     * @return Current calibration data
     */
    static CalibrationData getCalibration();
    
    /**
     * Set calibration data
     * @param cal Calibration data to set
     */
    static void setCalibration(const CalibrationData& cal);
    
    /**
     * Check if sensors are ready for reading
     * @return true if ready
     */
    static bool isReady();
    
    /**
     * Get sensor status string
     * @return Status string
     */
    static String getStatusString();
    
    /**
     * Generate demo data for testing
     * @param data Output demo data
     */
    static void generateDemoData(SensorData& data);
    
private:
    static bool initializeO2Sensor();
    static bool initializeCO2Sensor();
    static bool initializePressureSensor();
    static bool initializeAmbientSensor();
    static float calculateFlowRate(float pressurePa);
};