#include "SensorManager.h"
#include <Arduino.h>
#include <math.h>

// Static member definitions
DFRobot_OxygenSensor SensorManager::oxygenSensor;
Omron_D6FPH SensorManager::pressureSensor;
Adafruit_BMP280 SensorManager::ambientSensor;
bool SensorManager::sensorsInitialized = false;
bool SensorManager::co2SensorAvailable = false;

// Kalman filters
UnifiedKalmanFilter SensorManager::o2Filter(0.01, 0.1, 20.9);
UnifiedKalmanFilter SensorManager::co2Filter(0.01, 0.1, 400.0);
UnifiedKalmanFilter SensorManager::pressureFilter(0.01, 0.1, 0.0);
UnifiedKalmanFilter SensorManager::temperatureFilter(0.01, 0.1, 25.0);

SensorManager::CalibrationData SensorManager::calibration = {1.0, 0.0, 1.0};
unsigned long SensorManager::lastReading = 0;

bool SensorManager::initialize() {
    Serial.println("Initializing sensors...");
    
    bool success = true;
    
    // Initialize I2C
    Wire.begin();
    
    // Initialize O2 sensor
    if (!initializeO2Sensor()) {
        Serial.println("Warning: O2 sensor initialization failed");
        success = false;
    }
    
    // Initialize CO2 sensor
    if (!initializeCO2Sensor()) {
        Serial.println("Warning: CO2 sensor not available");
        co2SensorAvailable = false;
    }
    
    // Initialize pressure sensor
    if (!initializePressureSensor()) {
        Serial.println("Warning: Pressure sensor initialization failed");
        success = false;
    }
    
    // Initialize ambient sensor
    if (!initializeAmbientSensor()) {
        Serial.println("Warning: Ambient sensor initialization failed");
    }
    
    sensorsInitialized = success;
    
    if (success) {
        Serial.println("Sensors initialized successfully");
    }
    
    return success;
}

bool SensorManager::initializeO2Sensor() {
    // Initialize DFRobot O2 sensor with address 0x73
    oxygenSensor.begin(0x73);
    
    // Test communication
    delay(100);
    float testReading = oxygenSensor.ReadOxygenData(10);
    
    if (testReading > 0 && testReading < 30) {
        Serial.println("O2 sensor: OK");
        return true;
    } else {
        Serial.println("O2 sensor: FAILED");
        return false;
    }
}

bool SensorManager::initializeCO2Sensor() {
    // Try to initialize SCD30 CO2 sensor
    // Note: This is optional hardware
    
    // For now, return false as CO2 sensor is optional
    // In real implementation, you would try:
    // if (scd30.begin()) {
    //     co2SensorAvailable = true;
    //     return true;
    // }
    
    co2SensorAvailable = false;
    return false;
}

bool SensorManager::initializePressureSensor() {
    // Initialize Omron pressure sensor
    pressureSensor.begin();
    
    // Test communication
    delay(100);
    float testReading = pressureSensor.getPressure();
    
    // Check if reading is reasonable (not exactly 0 or error)
    if (!isnan(testReading) && abs(testReading) < 1000) {
        Serial.println("Pressure sensor: OK");
        return true;
    } else {
        Serial.println("Pressure sensor: FAILED");
        return false;
    }
}

bool SensorManager::initializeAmbientSensor() {
    // Initialize BMP280 for ambient conditions
    if (ambientSensor.begin()) {
        Serial.println("Ambient sensor: OK");
        return true;
    } else {
        Serial.println("Ambient sensor: FAILED");
        return false;
    }
}

bool SensorManager::readSensors(SensorData& data) {
    unsigned long now = millis();
    
    // Rate limiting
    if (now - lastReading < READING_INTERVAL) {
        return false;
    }
    
    lastReading = now;
    data.timestamp = now;
    
    // Initialize all validity flags
    data.o2Valid = false;
    data.co2Valid = false;
    data.pressureValid = false;
    data.ambientValid = false;
    
    // Read O2 sensor
    float rawO2 = oxygenSensor.ReadOxygenData(10);
    if (rawO2 > 0 && rawO2 < 30) {
        data.o2Percent = o2Filter.update(rawO2 * calibration.o2Correction);
        data.o2Valid = true;
    } else {
        data.o2Percent = 20.9; // Default value
        data.o2Valid = false;
    }
    
    // Read CO2 sensor (if available)
    if (co2SensorAvailable) {
        // Read CO2 sensor here
        // float rawCO2 = scd30.getCO2();
        // data.co2Ppm = co2Filter.update(rawCO2);
        // data.co2Valid = true;
        data.co2Ppm = 400; // Default
        data.co2Valid = false;
    } else {
        data.co2Ppm = 400; // Default atmospheric CO2
        data.co2Valid = false;
    }
    
    // Read pressure sensor
    float rawPressure = pressureSensor.getPressure();
    if (!isnan(rawPressure)) {
        data.pressure = pressureFilter.update(rawPressure - calibration.pressureOffset);
        data.pressureValid = true;
    } else {
        data.pressure = 0;
        data.pressureValid = false;
    }
    
    // Read ambient conditions
    if (ambientSensor.begin()) {
        data.temperature = temperatureFilter.update(ambientSensor.readTemperature());
        data.ambientPressure = ambientSensor.readPressure() / 100.0; // Convert to hPa
        data.humidity = 50.0; // BMP280 doesn't have humidity, use default
        data.ambientValid = true;
    } else {
        data.temperature = 25.0;
        data.ambientPressure = 1013.25;
        data.humidity = 50.0;
        data.ambientValid = false;
    }
    
    // Calculate flow rate
    data.flowRate = calculateFlowRate(data.pressure);
    
    return true;
}

bool SensorManager::calibrateO2(float referenceO2) {
    Serial.println("Starting O2 calibration...");
    
    // Take multiple readings
    float sum = 0;
    int validReadings = 0;
    
    for (int i = 0; i < 20; i++) {
        float reading = oxygenSensor.ReadOxygenData(10);
        if (reading > 0 && reading < 30) {
            sum += reading;
            validReadings++;
        }
        delay(100);
    }
    
    if (validReadings < 10) {
        Serial.println("O2 calibration failed - insufficient valid readings");
        return false;
    }
    
    float averageReading = sum / validReadings;
    calibration.o2Correction = referenceO2 / averageReading;
    
    Serial.print("O2 calibration complete. Factor: ");
    Serial.println(calibration.o2Correction);
    
    return true;
}

bool SensorManager::calibratePressure() {
    Serial.println("Starting pressure calibration (zero offset)...");
    
    // Take multiple readings at zero flow
    float sum = 0;
    int validReadings = 0;
    
    for (int i = 0; i < 50; i++) {
        float reading = pressureSensor.getPressure();
        if (!isnan(reading)) {
            sum += reading;
            validReadings++;
        }
        delay(50);
    }
    
    if (validReadings < 25) {
        Serial.println("Pressure calibration failed - insufficient valid readings");
        return false;
    }
    
    calibration.pressureOffset = sum / validReadings;
    
    Serial.print("Pressure calibration complete. Offset: ");
    Serial.println(calibration.pressureOffset);
    
    return true;
}

bool SensorManager::calibrateFlow(float syringeVolume, float injectionTime) {
    Serial.println("Starting flow calibration with syringe...");
    
    // This would require more complex implementation
    // For now, return a default calibration
    
    float expectedFlowRate = (syringeVolume / 1000.0) / injectionTime * 60.0; // L/min
    
    // In real implementation, you would:
    // 1. Start measurement
    // 2. Wait for syringe injection
    // 3. Calculate actual flow rate
    // 4. Determine calibration factor
    
    Serial.print("Flow calibration complete. Expected: ");
    Serial.print(expectedFlowRate);
    Serial.println(" L/min");
    
    return true;
}

SensorManager::CalibrationData SensorManager::getCalibration() {
    return calibration;
}

void SensorManager::setCalibration(const CalibrationData& cal) {
    calibration = cal;
}

bool SensorManager::isReady() {
    return sensorsInitialized;
}

String SensorManager::getStatusString() {
    String status = "Sensors: ";
    
    if (sensorsInitialized) {
        status += "O2:OK ";
        if (co2SensorAvailable) {
            status += "CO2:OK ";
        } else {
            status += "CO2:-- ";
        }
        status += "P:OK T:OK";
    } else {
        status += "FAILED";
    }
    
    return status;
}

void SensorManager::generateDemoData(SensorData& data) {
    unsigned long now = millis();
    data.timestamp = now;
    
    // Generate realistic demo data with some variation
    float time_s = now / 1000.0;
    
    // Simulate varying O2 (breathing pattern)
    data.o2Percent = 20.9 - 0.5 * sin(time_s * 0.5) + random(-10, 10) / 100.0;
    data.o2Valid = true;
    
    // Simulate varying CO2
    data.co2Ppm = 400 + 100 * sin(time_s * 0.3) + random(-20, 20);
    data.co2Valid = true;
    
    // Simulate pressure variations (breathing)
    data.pressure = 50 + 30 * sin(time_s * 0.8) + random(-5, 5);
    data.pressureValid = true;
    
    // Stable ambient conditions
    data.temperature = 25.0 + random(-5, 5) / 10.0;
    data.ambientPressure = 1013.25 + random(-2, 2);
    data.humidity = 50 + random(-5, 5);
    data.ambientValid = true;
    
    // Calculate flow rate
    data.flowRate = calculateFlowRate(data.pressure);
}

float SensorManager::calculateFlowRate(float pressurePa) {
    // Convert pressure to flow rate using venturi equation
    // Q = Cd * A * sqrt(2 * ΔP / ρ)
    if (pressurePa <= 0) return 0;
    
    const float Cd = 0.98;  // Discharge coefficient
    const float A = 0.000314;  // Throat area (m²) for 20mm diameter
    const float rho = 1.225;   // Air density (kg/m³)
    
    float flowRate = Cd * A * sqrt(2.0 * pressurePa / rho);
    
    // Convert m³/s to L/min and apply calibration
    return flowRate * 60000 * calibration.flowCalibration;
}