#include "CoreManager.h"
#include "SensorManager.h"
#include "MetabolicCalculator.h"
#include "DisplayManager.h"
#include "ConnectivityManager.h"
#include "config.h"

// Static member definitions
SemaphoreHandle_t CoreManager::xDataMutex = NULL;
SemaphoreHandle_t CoreManager::xDisplayMutex = NULL;
SemaphoreHandle_t CoreManager::xSensorMutex = NULL;
TaskHandle_t CoreManager::sensorTaskHandle = NULL;
TaskHandle_t CoreManager::uiTaskHandle = NULL;
TaskHandle_t CoreManager::watchdogTaskHandle = NULL;
bool CoreManager::systemHealthy = true;
unsigned long CoreManager::lastSensorUpdate = 0;
unsigned long CoreManager::lastUIUpdate = 0;

void CoreManager::initialize() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Initializing VO2Smart Core Manager...");
    
    // Create synchronization primitives
    xDataMutex = xSemaphoreCreateMutex();
    xDisplayMutex = xSemaphoreCreateMutex();
    xSensorMutex = xSemaphoreCreateMutex();
    
    if (!xDataMutex || !xDisplayMutex || !xSensorMutex) {
        Serial.println("ERROR: Failed to create mutexes!");
        systemHealthy = false;
        return;
    }
    
    // Initialize system components
    if (!SensorManager::begin()) {
        Serial.println("ERROR: Failed to initialize sensors!");
        systemHealthy = false;
        return;
    }
    
    #if USE_TFT_DISPLAY
    DisplayManager::initialize();
    #endif
    
    #if USE_BLE || USE_WIFI
    ConnectivityManager::Config connConfig = {
        .enableBLE = USE_BLE,
        .enableWiFi = USE_WIFI,
        .dualMode = DUAL_CONNECTIVITY,
        .deviceName = BLE_DEVICE_NAME,
        .wifiSSID = WIFI_DEFAULT_SSID,
        .wifiPassword = WIFI_DEFAULT_PASSWORD,
        .fallbackSSID = nullptr,
        .fallbackPassword = nullptr,
        .webServerPort = WEB_SERVER_PORT,
        .bleAdvertisingInterval = BLE_ADVERTISING_INTERVAL,
        .reconnectInterval = WIFI_RECONNECT_INTERVAL,
        .maxReconnectAttempts = WIFI_MAX_RECONNECT_ATTEMPTS,
        .enableEncryption = ENABLE_ENCRYPTION
    };
    ConnectivityManager::begin(connConfig);
    #endif
    
    // Configure metabolic calculator
    MetabolicCalculator::configure(75.0f, 175.0f, 25, true); // Default values
    
    Serial.println("Core Manager initialized successfully");
    systemHealthy = true;
}

void CoreManager::createTasks() {
    if (!systemHealthy) {
        Serial.println("ERROR: Cannot create tasks - system not healthy!");
        return;
    }
    
    Serial.println("Creating FreeRTOS tasks...");
    
    // Create sensor task on Core 0 (dedicated processing core)
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensorTaskHandle,
        CORE_SENSOR_DATA
    );
    
    if (!sensorTaskHandle) {
        Serial.println("ERROR: Failed to create sensor task!");
        systemHealthy = false;
        return;
    }
    
    // Create UI task on Core 1 (communication and display core)
    xTaskCreatePinnedToCore(
        uiTask,
        "UITask", 
        UI_TASK_STACK,
        NULL,
        UI_TASK_PRIORITY,
        &uiTaskHandle,
        CORE_UI_COMM
    );
    
    if (!uiTaskHandle) {
        Serial.println("ERROR: Failed to create UI task!");
        systemHealthy = false;
        return;
    }
    
    #if ENABLE_WATCHDOG
    // Create watchdog task
    xTaskCreatePinnedToCore(
        watchdogTask,
        "WatchdogTask",
        WATCHDOG_TASK_STACK,
        NULL,
        WATCHDOG_TASK_PRIORITY,
        &watchdogTaskHandle,
        CORE_UI_COMM
    );
    #endif
    
    Serial.println("All tasks created successfully");
}

void CoreManager::sensorTask(void* params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_TASK_FREQ_HZ);
    
    Serial.println("Sensor task started on Core 0");
    
    while (true) {
        unsigned long startTime = micros();
        
        // Read sensor data with mutex protection
        SensorManager::SensorData sensorData;
        bool sensorSuccess = false;
        
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50))) {
            sensorSuccess = SensorManager::readSensors(sensorData);
            xSemaphoreGive(xSensorMutex);
        }
        
        if (sensorSuccess) {
            // Perform metabolic calculations with data mutex protection
            if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(50))) {
                MetabolicCalculator::MetabolicData metabolicData;
                bool calcSuccess = MetabolicCalculator::calculate(sensorData, metabolicData);
                
                if (calcSuccess) {
                    // Store data for UI and connectivity tasks
                    // This would typically update a global data structure
                    lastSensorUpdate = millis();
                }
                
                xSemaphoreGive(xDataMutex);
            }
        } else {
            handleTaskError("SensorRead");
        }
        
        #if ENABLE_PERFORMANCE_MON
        unsigned long taskTime = micros() - startTime;
        if (taskTime > 40000) { // 40ms is getting close to our 50ms budget
            Serial.printf("WARNING: Sensor task took %lu us\n", taskTime);
        }
        #endif
        
        // Maintain precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void CoreManager::uiTask(void* params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / UI_TASK_FREQ_HZ);
    
    Serial.println("UI task started on Core 1");
    
    while (true) {
        unsigned long startTime = micros();
        
        // Update display with mutex protection
        #if USE_TFT_DISPLAY
        if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(50))) {
            DisplayManager::update();
            xSemaphoreGive(xDisplayMutex);
            lastUIUpdate = millis();
        }
        #endif
        
        // Handle connectivity
        #if USE_BLE || USE_WIFI
        ConnectivityManager::update();
        ConnectivityManager::handleClients();
        #endif
        
        #if ENABLE_PERFORMANCE_MON
        unsigned long taskTime = micros() - startTime;
        if (taskTime > 80000) { // 80ms is getting close to our 100ms budget
            Serial.printf("WARNING: UI task took %lu us\n", taskTime);
        }
        #endif
        
        // Maintain precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void CoreManager::watchdogTask(void* params) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Check every second
    
    Serial.println("Watchdog task started");
    
    while (true) {
        unsigned long currentTime = millis();
        
        // Check if sensor task is responding
        if (currentTime - lastSensorUpdate > HEALTH_CHECK_TIMEOUT_MS) {
            Serial.println("WARNING: Sensor task not responding!");
            systemHealthy = false;
        }
        
        // Check if UI task is responding
        if (currentTime - lastUIUpdate > HEALTH_CHECK_TIMEOUT_MS) {
            Serial.println("WARNING: UI task not responding!");
            systemHealthy = false;
        }
        
        // Check free memory
        #if ENABLE_MEMORY_DEBUG
        size_t freeHeap = esp_get_free_heap_size();
        if (freeHeap < 10000) { // Less than 10KB free
            Serial.printf("WARNING: Low memory - %zu bytes free\n", freeHeap);
        }
        #endif
        
        // System recovery if needed
        if (!systemHealthy) {
            Serial.println("Attempting system recovery...");
            // Implement recovery procedures here
            vTaskDelay(pdMS_TO_TICKS(1000));
            systemHealthy = true; // Reset for next check
        }
        
        vTaskDelay(xFrequency);
    }
}

void CoreManager::shutdown() {
    Serial.println("Shutting down Core Manager...");
    
    // Suspend all tasks
    if (sensorTaskHandle) vTaskSuspend(sensorTaskHandle);
    if (uiTaskHandle) vTaskSuspend(uiTaskHandle);
    if (watchdogTaskHandle) vTaskSuspend(watchdogTaskHandle);
    
    // Shutdown components
    SensorManager::shutdown();
    #if USE_TFT_DISPLAY
    DisplayManager::shutdown();
    #endif
    #if USE_BLE || USE_WIFI
    ConnectivityManager::shutdown();
    #endif
    
    // Clean up mutexes
    if (xDataMutex) vSemaphoreDelete(xDataMutex);
    if (xDisplayMutex) vSemaphoreDelete(xDisplayMutex);
    if (xSensorMutex) vSemaphoreDelete(xSensorMutex);
    
    Serial.println("Core Manager shutdown complete");
}

bool CoreManager::isSystemHealthy() {
    return systemHealthy;
}

void CoreManager::handleTaskError(const char* taskName) {
    Serial.printf("ERROR in task: %s\n", taskName);
    systemHealthy = false;
    
    // Could implement specific recovery procedures here
    // For now, just log the error
}

void CoreManager::suspendSensorTask() {
    if (sensorTaskHandle) {
        vTaskSuspend(sensorTaskHandle);
    }
}

void CoreManager::resumeSensorTask() {
    if (sensorTaskHandle) {
        vTaskResume(sensorTaskHandle);
    }
}

void CoreManager::suspendUITask() {
    if (uiTaskHandle) {
        vTaskSuspend(uiTaskHandle);
    }
}

void CoreManager::resumeUITask() {
    if (uiTaskHandle) {
        vTaskResume(uiTaskHandle);
    }
}