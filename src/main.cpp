/**
 * VO2Smart - Advanced ESP32-based Metabolic Monitoring System
 * 
 * Features:
 * - Real-time VO2/VCO2 measurement and analysis
 * - Ventilatory threshold detection (VT1/VT2)
 * - Dynamic visualization with TFT display
 * - Dual connectivity (BLE/WiFi) with auto-reconnection
 * - Optimized dual-core architecture
 * - Advanced sensor fusion and error handling
 * 
 * Hardware: ESP32 with various metabolic sensors
 * Version: 2.1.0
 * Author: VO2Smart Development Team
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "CoreManager.h"
#include "SensorManager.h"
#include "MetabolicCalculator.h"
#include "DisplayManager.h"
#include "ConnectivityManager.h"

// Global system state
volatile bool systemRunning = true;
volatile bool calibrationMode = false;
unsigned long systemStartTime = 0;

// Button handling
volatile bool powerButtonPressed = false;
volatile bool modeButtonPressed = false;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// Function prototypes
void setupHardware();
void handleButtons();
void handleSerialCommands();
void performSystemDiagnostics();
void enterCalibrationMode();
void exitCalibrationMode();
void handlePowerManagement();
void handleEmergencyShutdown();

// Interrupt handlers
void IRAM_ATTR powerButtonISR() {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE_MS) {
        powerButtonPressed = true;
        lastButtonPress = millis();
    }
}

void IRAM_ATTR modeButtonISR() {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE_MS) {
        modeButtonPressed = true;
        lastButtonPress = millis();
    }
}

void setup() {
    systemStartTime = millis();
    
    #if ENABLE_SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() - systemStartTime < 3000) {
        // Wait for serial connection with timeout
        delay(10);
    }
    
    Serial.println("\n" "=" * 50);
    Serial.println("VO2Smart Metabolic Monitor v" FIRMWARE_VERSION);
    Serial.println("Hardware v" HARDWARE_VERSION);
    Serial.println("Build: " BUILD_DATE " " BUILD_TIME);
    Serial.println("=" * 50);
    #endif
    
    // Hardware initialization
    setupHardware();
    
    // Initialize core management system
    CoreManager::initialize();
    
    if (!CoreManager::isSystemHealthy()) {
        Serial.println("CRITICAL ERROR: System initialization failed!");
        #if USE_TFT_DISPLAY
        DisplayManager::showError("System Error", "Initialization failed - check sensors", 0);
        #endif
        handleEmergencyShutdown();
        return;
    }
    
    // Create and start FreeRTOS tasks
    CoreManager::createTasks();
    
    #if ENABLE_SERIAL_DEBUG
    Serial.println("Setup complete - entering main loop");
    Serial.println("Available commands: 'diag', 'cal', 'reset', 'info'");
    #endif
    
    // Show startup success
    #if USE_TFT_DISPLAY
    DisplayManager::showSuccess("System Ready", 2000);
    #endif
    
    #if PIN_BUZZER > 0
    // Startup beep sequence
    digitalWrite(PIN_BUZZER, HIGH);
    delay(100);
    digitalWrite(PIN_BUZZER, LOW);
    delay(50);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(100);
    digitalWrite(PIN_BUZZER, LOW);
    #endif
}

void loop() {
    // Main loop runs on Core 1 (same as Arduino main task)
    // Most work is done in FreeRTOS tasks, this handles user interaction
    
    static unsigned long lastStatusCheck = 0;
    static unsigned long lastPerformanceCheck = 0;
    
    // Handle user input
    handleButtons();
    
    #if ENABLE_SERIAL_DEBUG
    handleSerialCommands();
    #endif
    
    // Periodic status checks
    if (millis() - lastStatusCheck > 10000) { // Every 10 seconds
        if (!CoreManager::isSystemHealthy()) {
            Serial.println("WARNING: System health check failed!");
            #if USE_TFT_DISPLAY
            DisplayManager::showWarning("System Warning", 3000);
            #endif
        }
        lastStatusCheck = millis();
    }
    
    #if ENABLE_PERFORMANCE_MON
    // Performance monitoring
    if (millis() - lastPerformanceCheck > 30000) { // Every 30 seconds
        size_t freeHeap = esp_get_free_heap_size();
        size_t minFreeHeap = esp_get_minimum_free_heap_size();
        
        Serial.printf("Memory - Free: %zu bytes, Min Free: %zu bytes\n", 
                     freeHeap, minFreeHeap);
        
        lastPerformanceCheck = millis();
    }
    #endif
    
    // Power management
    #if ENABLE_POWER_SAVE
    handlePowerManagement();
    #endif
    
    // Small delay to prevent excessive CPU usage
    delay(50);
}

void setupHardware() {
    Serial.println("Initializing hardware...");
    
    // Configure I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); // 400kHz for faster sensor communication
    
    // Configure GPIO pins
    #if PIN_BUTTON_POWER > 0
    pinMode(PIN_BUTTON_POWER, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_POWER), powerButtonISR, FALLING);
    #endif
    
    #if PIN_BUTTON_MODE > 0
    pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_MODE), modeButtonISR, FALLING);
    #endif
    
    #if PIN_BUZZER > 0
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    #endif
    
    #if PIN_LED_STATUS > 0
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, HIGH); // Indicate system starting
    #endif
    
    Serial.println("Hardware initialization complete");
}

void handleButtons() {
    if (powerButtonPressed) {
        powerButtonPressed = false;
        
        Serial.println("Power button pressed");
        
        if (calibrationMode) {
            exitCalibrationMode();
        } else {
            // Toggle system power state
            systemRunning = !systemRunning;
            
            if (systemRunning) {
                CoreManager::resumeSensorTask();
                CoreManager::resumeUITask();
                #if USE_TFT_DISPLAY
                DisplayManager::showSuccess("System Resumed", 1000);
                #endif
                Serial.println("System resumed");
            } else {
                CoreManager::suspendSensorTask();
                CoreManager::suspendUITask();
                #if USE_TFT_DISPLAY
                DisplayManager::showWarning("System Paused", 1000);
                #endif
                Serial.println("System paused");
            }
        }
    }
    
    if (modeButtonPressed) {
        modeButtonPressed = false;
        
        Serial.println("Mode button pressed");
        
        if (!calibrationMode) {
            enterCalibrationMode();
        } else {
            // Cycle through calibration options
            // This would be implemented based on current calibration state
            Serial.println("Cycling calibration mode");
        }
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command == "diag") {
            performSystemDiagnostics();
        } else if (command == "cal") {
            if (calibrationMode) {
                exitCalibrationMode();
            } else {
                enterCalibrationMode();
            }
        } else if (command == "reset") {
            Serial.println("Resetting system...");
            ESP.restart();
        } else if (command == "info") {
            Serial.println("\nSystem Information:");
            Serial.println("==================");
            Serial.printf("Firmware: v%s\n", FIRMWARE_VERSION);
            Serial.printf("Hardware: v%s\n", HARDWARE_VERSION);
            Serial.printf("Uptime: %lu seconds\n", (millis() - systemStartTime) / 1000);
            Serial.printf("Free Heap: %zu bytes\n", esp_get_free_heap_size());
            Serial.printf("System Healthy: %s\n", CoreManager::isSystemHealthy() ? "Yes" : "No");
            
            #if USE_BLE || USE_WIFI
            auto connState = ConnectivityManager::getConnectionState();
            Serial.printf("BLE Status: %d\n", connState.bleStatus);
            Serial.printf("WiFi Status: %d\n", connState.wifiStatus);
            #endif
        } else if (command.startsWith("config ")) {
            // Handle configuration commands
            Serial.println("Configuration commands not implemented yet");
        } else {
            Serial.println("Unknown command. Available: diag, cal, reset, info");
        }
    }
}

void performSystemDiagnostics() {
    Serial.println("\nPerforming system diagnostics...");
    Serial.println("=================================");
    
    // Test sensors
    bool sensorHealth = SensorManager::performSelfTest();
    Serial.printf("Sensor Health: %s\n", sensorHealth ? "PASS" : "FAIL");
    
    // Test connectivity
    #if USE_BLE || USE_WIFI
    bool connHealth = ConnectivityManager::performConnectionDiagnostics();
    Serial.printf("Connectivity Health: %s\n", connHealth ? "PASS" : "FAIL");
    #endif
    
    // Test memory
    size_t freeHeap = esp_get_free_heap_size();
    bool memoryOK = freeHeap > 20000; // At least 20KB free
    Serial.printf("Memory Health: %s (%zu bytes free)\n", 
                 memoryOK ? "PASS" : "FAIL", freeHeap);
    
    // Test tasks
    bool tasksOK = CoreManager::isSystemHealthy();
    Serial.printf("Task Health: %s\n", tasksOK ? "PASS" : "FAIL");
    
    Serial.println("Diagnostics complete");
    
    #if USE_TFT_DISPLAY
    if (sensorHealth && memoryOK && tasksOK) {
        DisplayManager::showSuccess("Diagnostics PASS", 2000);
    } else {
        DisplayManager::showError("Diagnostics FAIL", "Check serial output", 5000);
    }
    #endif
}

void enterCalibrationMode() {
    calibrationMode = true;
    Serial.println("Entering calibration mode...");
    
    #if USE_TFT_DISPLAY
    DisplayManager::setScreen(DisplayManager::CALIBRATION_MODE);
    DisplayManager::showWarning("Calibration Mode", 2000);
    #endif
    
    // Pause normal operations during calibration
    CoreManager::suspendSensorTask();
}

void exitCalibrationMode() {
    calibrationMode = false;
    Serial.println("Exiting calibration mode...");
    
    #if USE_TFT_DISPLAY
    DisplayManager::setScreen(DisplayManager::MAIN_METRICS);
    DisplayManager::showSuccess("Calibration Complete", 2000);
    #endif
    
    // Resume normal operations
    CoreManager::resumeSensorTask();
}

void handlePowerManagement() {
    // This would implement battery monitoring and power saving features
    // For now, just a placeholder
    static unsigned long lastPowerCheck = 0;
    
    if (millis() - lastPowerCheck > 60000) { // Check every minute
        // Check battery level, implement power saving if needed
        // This would be sensor-specific implementation
        lastPowerCheck = millis();
    }
}

void handleEmergencyShutdown() {
    Serial.println("EMERGENCY SHUTDOWN INITIATED");
    
    #if PIN_LED_STATUS > 0
    // Flash LED to indicate error
    for (int i = 0; i < 10; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        delay(100);
        digitalWrite(PIN_LED_STATUS, LOW);
        delay(100);
    }
    #endif
    
    #if PIN_BUZZER > 0
    // Error tone
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(500);
        digitalWrite(PIN_BUZZER, LOW);
        delay(200);
    }
    #endif
    
    // Attempt graceful shutdown
    CoreManager::shutdown();
    
    // Enter infinite loop to prevent further execution
    while (true) {
        delay(1000);
    }
}