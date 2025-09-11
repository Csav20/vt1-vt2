/**
 * VO2Smart - ESP32 Integrated VO2Max System
 * Refactored Version 3.1 - Modular Implementation
 * 
 * Key Improvements:
 * - Removed BluetoothSerial conflict (BLE only)
 * - Unified Kalman filter implementation
 * - Optimized display rendering (no flickering)
 * - Modular code structure
 * - English naming standardization
 * - Navigation support for DEMO and VO2 modes
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TFT_eSPI.h>
#include <CircularBuffer.hpp>

// Project modules
#include "KalmanFilter.h"
#include "DisplayManager.h"
#include "BluetoothManager.h"
#include "SensorManager.h"
#include "WiFiManager.h"

// Device configuration
#define DEVICE_VERSION "V3.1 2025/04/02"
#define DEVICE_NAME "VO2Smart"

// VO2 calculation methods
#define VO2_METHOD_VSLOPE 0
#define VO2_METHOD_WASSERMAN 1  
#define VO2_METHOD_BEAVER 2
#define VO2_METHOD_DICKHUTH 3

// Hardware pins
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_PIN1 0
#define BUTTON_PIN2 35

// Sensor configuration
#define OXYGEN_I2C_ADDRESS 0x73  // ADDRESS_3 equivalent
#define OXYGEN_COLLECT_NUMBER 10

// Timing constants
#define SENSOR_SAMPLING_MS 50    // 20Hz
#define UI_REFRESH_MS 100        // 10Hz
#define VO2_CALCULATION_MS 5000  // 0.2Hz

// Scientific constants
#define FI02 20.93              // Ambient O2 percentage
#define FICO2 0.0004           // Ambient CO2 percentage

// WiFi configuration
const char* ssid = "VO2Max_Network";
const char* password = "vo2smart2025";

// Global variables
struct Settings {
    float weight;
    float o2Correction;
    float pressureOffset;
    float flowCalibration;
    int vo2Method;
    bool demoMode;
    bool wifiEnabled;
    bool bleEnabled;
} settings;

struct LiveData {
    float vo2;
    float vco2;
    float rer;
    float o2Percent;
    float co2Ppm;
    float pressure;
    float temperature;
    float humidity;
    float batteryVoltage;
    bool sensorsReady;
    unsigned long timestamp;
} liveData;

// System state
bool systemReady = false;
bool demoMode = true;
DisplayManager::NavigationMode currentMode = DisplayManager::MODE_NORMAL;

// Button state tracking
bool button1Pressed = false;
bool button2Pressed = false;
unsigned long button1PressTime = 0;
unsigned long button2PressTime = 0;

// Timing
unsigned long lastSensorReading = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastVO2Calculation = 0;

// Function prototypes
void setup();
void loop();
void initializeSystem();
void readSensors();
void updateDisplay();
void calculateVO2();
void handleButtons();
void handleNavigation();
void loadSettings();
void saveSettings();
void enterDemoMode();
void exitDemoMode();
float calculateFlowRate(float pressure);
float calculateVO2(float flowRate, float o2Percent);
float getBatteryVoltage();

void setup() {
    Serial.begin(115200);
    Serial.println("VO2Smart " DEVICE_VERSION " - Initializing...");
    
    // Initialize system
    initializeSystem();
    
    Serial.println("System ready!");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Read sensors at 20Hz
    if (currentTime - lastSensorReading >= SENSOR_SAMPLING_MS) {
        readSensors();
        lastSensorReading = currentTime;
    }
    
    // Update display at 10Hz
    if (currentTime - lastDisplayUpdate >= UI_REFRESH_MS) {
        updateDisplay();
        lastDisplayUpdate = currentTime;
    }
    
    // Calculate VO2 at 0.2Hz
    if (currentTime - lastVO2Calculation >= VO2_CALCULATION_MS) {
        calculateVO2();
        lastVO2Calculation = currentTime;
    }
    
    // Handle button input
    handleButtons();
    
    // Handle navigation
    handleNavigation();
    
    // Handle BLE events
    BluetoothManager::handleEvents();
    
    // Handle WiFi events
    if (settings.wifiEnabled) {
        WiFiManager::handleEvents();
    }
    
    // Small delay to prevent watchdog issues
    delay(1);
}

void initializeSystem() {
    // Initialize pins
    pinMode(ADC_EN, OUTPUT);
    pinMode(BUTTON_PIN1, INPUT_PULLUP);
    pinMode(BUTTON_PIN2, INPUT_PULLUP);
    digitalWrite(ADC_EN, HIGH);
    
    // Load settings from EEPROM
    loadSettings();
    
    // Initialize display
    if (!DisplayManager::initialize()) {
        Serial.println("Error: Display initialization failed");
        return;
    }
    
    // Show splash screen
    DisplayManager::setScreen(DisplayManager::SCREEN_SPLASH);
    
    // Initialize sensors
    if (!SensorManager::initialize()) {
        Serial.println("Warning: Some sensors failed to initialize");
    }
    
    // Initialize BLE (no BluetoothSerial conflict)
    if (settings.bleEnabled) {
        if (!BluetoothManager::initialize(DEVICE_NAME)) {
            Serial.println("Warning: BLE initialization failed");
        }
    }
    
    // Initialize WiFi if enabled
    if (settings.wifiEnabled) {
        WiFiManager::initialize(ssid, password);
        WiFiManager::connect();
        Serial.println("WiFi connection initiated...");
    }
    
    // Set initial display mode
    DisplayManager::setMode(settings.demoMode ? 
        DisplayManager::MODE_DEMO : DisplayManager::MODE_NORMAL);
    
    // Wait for splash screen
    delay(2000);
    
    // Switch to main screen
    DisplayManager::setScreen(DisplayManager::SCREEN_MAIN);
    
    systemReady = true;
}

void readSensors() {
    if (!systemReady) return;
    
    SensorManager::SensorData sensorData;
    
    if (settings.demoMode) {
        // Generate demo data
        SensorManager::generateDemoData(sensorData);
    } else {
        // Read real sensors
        if (!SensorManager::readSensors(sensorData)) {
            Serial.println("Warning: Sensor reading failed");
            return;
        }
    }
    
    // Update live data
    liveData.o2Percent = sensorData.o2Percent;
    liveData.co2Ppm = sensorData.co2Ppm;
    liveData.pressure = sensorData.pressure;
    liveData.temperature = sensorData.temperature;
    liveData.humidity = sensorData.humidity;
    liveData.timestamp = sensorData.timestamp;
    liveData.sensorsReady = sensorData.o2Valid && sensorData.pressureValid;
    
    // Update battery voltage
    liveData.batteryVoltage = getBatteryVoltage();
}

void updateDisplay() {
    if (!systemReady) return;
    
    // Prepare display data
    DisplayManager::DisplayData displayData;
    displayData.vo2 = liveData.vo2;
    displayData.vco2 = liveData.vco2;
    displayData.rer = liveData.rer;
    displayData.o2Percent = liveData.o2Percent;
    displayData.co2Ppm = liveData.co2Ppm;
    displayData.pressure = liveData.pressure;
    displayData.temperature = liveData.temperature;
    displayData.humidity = liveData.humidity;
    displayData.batteryVoltage = liveData.batteryVoltage;
    displayData.bleConnected = BluetoothManager::isConnected();
    displayData.wifiConnected = WiFiManager::isConnected();
    displayData.time = String(millis() / 1000);
    displayData.mode = currentMode;
    
    // Update display (optimized - no flickering)
    DisplayManager::update(displayData);
}

void calculateVO2() {
    if (!systemReady || !liveData.sensorsReady) return;
    
    // Calculate flow rate from pressure
    float flowRate = calculateFlowRate(liveData.pressure);
    
    // Calculate VO2 based on selected method
    switch (settings.vo2Method) {
        case VO2_METHOD_VSLOPE:
        default:
            liveData.vo2 = calculateVO2(flowRate, liveData.o2Percent);
            break;
        case VO2_METHOD_WASSERMAN:
            // Implement Wasserman method
            liveData.vo2 = calculateVO2(flowRate, liveData.o2Percent) * 1.05; // Example
            break;
        case VO2_METHOD_BEAVER:
            // Implement Beaver method  
            liveData.vo2 = calculateVO2(flowRate, liveData.o2Percent) * 0.98; // Example
            break;
        case VO2_METHOD_DICKHUTH:
            // Implement Dickhuth method
            liveData.vo2 = calculateVO2(flowRate, liveData.o2Percent) * 1.02; // Example
            break;
    }
    
    // Calculate VCO2 if CO2 sensor available
    if (liveData.co2Ppm > 0) {
        liveData.vco2 = flowRate * (liveData.co2Ppm - FICO2) / 1000000.0 * 0.863;
        liveData.rer = liveData.vco2 / liveData.vo2;
    } else {
        liveData.vco2 = 0;
        liveData.rer = 0.85; // Default RER
    }
    
    // Send data via BLE if connected
    if (BluetoothManager::isConnected()) {
        String jsonData = "{";
        jsonData += "\"vo2\":" + String(liveData.vo2, 2) + ",";
        jsonData += "\"vco2\":" + String(liveData.vco2, 2) + ",";
        jsonData += "\"rer\":" + String(liveData.rer, 2) + ",";
        jsonData += "\"o2\":" + String(liveData.o2Percent, 1) + ",";
        jsonData += "\"co2\":" + String(liveData.co2Ppm, 0) + ",";
        jsonData += "\"pressure\":" + String(liveData.pressure, 1) + ",";
        jsonData += "\"temperature\":" + String(liveData.temperature, 1) + ",";
        jsonData += "\"timestamp\":" + String(liveData.timestamp);
        jsonData += "}";
        
        BluetoothManager::sendData(jsonData);
    }
}

void handleButtons() {
    // Read button states (active low)
    bool btn1State = !digitalRead(BUTTON_PIN1);
    bool btn2State = !digitalRead(BUTTON_PIN2);
    
    // Button 1 - Navigate screens
    if (btn1State && !button1Pressed) {
        button1Pressed = true;
        button1PressTime = millis();
        DisplayManager::nextScreen();
    } else if (!btn1State && button1Pressed) {
        button1Pressed = false;
    }
    
    // Button 2 - Change mode/settings
    if (btn2State && !button2Pressed) {
        button2Pressed = true;
        button2PressTime = millis();
        
        // Toggle demo mode
        settings.demoMode = !settings.demoMode;
        currentMode = settings.demoMode ? 
            DisplayManager::MODE_DEMO : DisplayManager::MODE_NORMAL;
        DisplayManager::setMode(currentMode);
        saveSettings();
    } else if (!btn2State && button2Pressed) {
        button2Pressed = false;
    }
    
    // Both buttons pressed - enter VO2 mode
    if (btn1State && btn2State) {
        currentMode = DisplayManager::MODE_VO2;
        DisplayManager::setMode(currentMode);
    }
}

void handleNavigation() {
    // Handle navigation based on current mode
    switch (currentMode) {
        case DisplayManager::MODE_DEMO:
            // Auto-cycle screens in demo mode
            static unsigned long lastScreenChange = 0;
            if (millis() - lastScreenChange > 5000) { // 5 second intervals
                DisplayManager::nextScreen();
                lastScreenChange = millis();
            }
            break;
            
        case DisplayManager::MODE_VO2:
            // VO2 mode specific navigation
            break;
            
        case DisplayManager::MODE_NORMAL:
        default:
            // Manual navigation only
            break;
    }
}

void loadSettings() {
    EEPROM.begin(512);
    
    // Check if settings exist (magic number)
    uint32_t magic;
    EEPROM.get(0, magic);
    
    if (magic == 0xDEADBEEF) {
        // Load existing settings
        EEPROM.get(4, settings);
    } else {
        // Initialize default settings
        settings.weight = 70.0;
        settings.o2Correction = 1.0;
        settings.pressureOffset = 0.0;
        settings.flowCalibration = 1.0;
        settings.vo2Method = VO2_METHOD_VSLOPE;
        settings.demoMode = true;
        settings.wifiEnabled = false;
        settings.bleEnabled = true;
        saveSettings();
    }
}

void saveSettings() {
    // Save magic number
    uint32_t magic = 0xDEADBEEF;
    EEPROM.put(0, magic);
    
    // Save settings
    EEPROM.put(4, settings);
    EEPROM.commit();
}

void enterDemoMode() {
    settings.demoMode = true;
    currentMode = DisplayManager::MODE_DEMO;
    DisplayManager::setMode(currentMode);
    saveSettings();
}

void exitDemoMode() {
    settings.demoMode = false;
    currentMode = DisplayManager::MODE_NORMAL;
    DisplayManager::setMode(currentMode);
    saveSettings();
}

float calculateFlowRate(float pressure) {
    // Convert pressure to flow rate using venturi equation
    // Q = Cd * A * sqrt(2 * ΔP / ρ)
    if (pressure <= 0) return 0;
    
    const float Cd = 0.98;  // Discharge coefficient
    const float A = 0.000314;  // Throat area (m²)
    const float rho = 1.225;   // Air density (kg/m³)
    
    float flowRate = Cd * A * sqrt(2.0 * pressure / rho);
    return flowRate * 60000; // Convert m³/s to L/min
}

float calculateVO2(float flowRate, float o2Percent) {
    // VO2 = VE * (FiO2 - FeO2) * correction factors
    if (flowRate <= 0 || o2Percent <= 0) return 0;
    
    float fiO2 = FI02 / 100.0;  // Inspired O2 fraction
    float feO2 = o2Percent / 100.0;  // Expired O2 fraction
    float vo2 = flowRate * (fiO2 - feO2) * 1000; // mL/min
    
    return vo2;
}

float getBatteryVoltage() {
    // Read battery voltage from ADC
    uint16_t v = analogRead(ADC_PIN);
    float voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (1100.0 / 1000.0);
    return voltage;
}