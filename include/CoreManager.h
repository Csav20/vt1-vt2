#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/**
 * @brief Core management system for dual-core ESP32 operations
 * 
 * Manages task distribution between cores:
 * - Core 0: Sensor reading and data processing (20Hz)
 * - Core 1: UI updates and communication (10Hz)
 */
class CoreManager {
public:
    static void initialize();
    static void createTasks();
    static void shutdown();
    
    // Task management
    static void suspendSensorTask();
    static void resumeSensorTask();
    static void suspendUITask();
    static void resumeUITask();
    
    // Error handling
    static bool isSystemHealthy();
    static void handleTaskError(const char* taskName);
    
private:
    static void sensorTask(void* params);
    static void uiTask(void* params);
    static void watchdogTask(void* params);
    
    // Synchronization primitives
    static SemaphoreHandle_t xDataMutex;
    static SemaphoreHandle_t xDisplayMutex;
    static SemaphoreHandle_t xSensorMutex;
    
    // Task handles for management
    static TaskHandle_t sensorTaskHandle;
    static TaskHandle_t uiTaskHandle;
    static TaskHandle_t watchdogTaskHandle;
    
    // System health monitoring
    static bool systemHealthy;
    static unsigned long lastSensorUpdate;
    static unsigned long lastUIUpdate;
    
    // Constants
    static const int SENSOR_TASK_PRIORITY = 2;
    static const int UI_TASK_PRIORITY = 3;
    static const int WATCHDOG_TASK_PRIORITY = 1;
    static const int SENSOR_STACK_SIZE = 10000;
    static const int UI_STACK_SIZE = 12000;
    static const int WATCHDOG_STACK_SIZE = 2048;
    static const int SENSOR_FREQUENCY_HZ = 20;
    static const int UI_FREQUENCY_HZ = 10;
    static const unsigned long HEALTH_CHECK_TIMEOUT_MS = 5000;
};