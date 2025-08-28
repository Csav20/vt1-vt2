#pragma once

// ============================================================================
// VO2Smart System Configuration
// ESP32-based Metabolic Monitoring System
// ============================================================================

// Hardware Configuration
#define ESP32_DUAL_CORE         1
#define CORE_SENSOR_DATA        0      // Core 0 for sensor processing
#define CORE_UI_COMM           1      // Core 1 for UI and communication

// Sensor Selection (uncomment to enable)
#define USE_MAX30102           1      // Heart rate and SpO2
#define USE_MAX30105           1      // NIRS and metabolite detection
#define USE_VL53L0X            1      // Time-of-flight distance sensor
#define USE_MLX90614           1      // Non-contact temperature sensor
#define USE_BMP280             1      // Environmental pressure/temperature
#define USE_QMI8658            1      // IMU (replaces MPU6050)

// Display Configuration
#define USE_TFT_DISPLAY        1
#define DISPLAY_WIDTH          240
#define DISPLAY_HEIGHT         320
#define DISPLAY_ROTATION       0

// Communication Configuration
#define USE_BLE                1
#define USE_WIFI               1
#define DUAL_CONNECTIVITY      1      // Allow simultaneous BLE and WiFi

// Pin Definitions
#define PIN_SDA                21
#define PIN_SCL                22
#define PIN_TFT_CS             5
#define PIN_TFT_DC             2
#define PIN_TFT_RST            4
#define PIN_BUTTON_POWER       0
#define PIN_BUTTON_MODE        15
#define PIN_BUZZER             13
#define PIN_LED_STATUS         2

// I2C Addresses
#define ADDR_MAX30102          0x57
#define ADDR_MAX30105          0x58   // Alternative address if conflict
#define ADDR_VL53L0X           0x29
#define ADDR_MLX90614          0x5A
#define ADDR_BMP280            0x77
#define ADDR_QMI8658           0x6B

// System Performance Configuration
#define SENSOR_TASK_FREQ_HZ    20     // Sensor reading frequency
#define UI_TASK_FREQ_HZ        10     // UI update frequency
#define METABOLIC_CALC_FREQ_HZ 5      // Metabolic calculation frequency

// Memory Configuration
#define SENSOR_TASK_STACK      10000
#define UI_TASK_STACK          12000
#define WATCHDOG_TASK_STACK    2048
#define DATA_BUFFER_SIZE       240    // 12 seconds at 20Hz
#define GRAPH_BUFFER_SIZE      1200   // 60 seconds at 20Hz

// Calculation Optimization Constants
#define VENTURI_AREA_M2        0.000012566f  // π * (2mm)² - Pre-calculated
#define VENTURI_COEFFICIENT    0.98f         // Discharge coefficient
#define AIR_DENSITY_STP        1.225f        // kg/m³ at 20°C, 1 atm
#define O2_DENSITY_STP         1.429f        // kg/m³ at STP
#define CO2_DENSITY_STP        1.977f        // kg/m³ at STP
#define GAS_CONSTANT_R         8.314f        // J/(mol·K)

// Metabolic Calculation Constants
#define STPD_FACTOR            0.8630f       // Pre-calculated STPD correction
#define BTPS_FACTOR            1.1020f       // Pre-calculated BTPS correction
#define CALORIC_EQUIV_O2       4.825f        // kcal/L O2 (average RER=0.85)
#define CALORIC_EQUIV_CO2      6.757f        // kcal/L CO2
#define O2_EXTRACTION_COEFF    0.0565f       // Pre-calculated extraction coefficient

// Threshold Detection Parameters
#define VT1_DETECTION_WINDOW   60            // seconds
#define VT2_DETECTION_WINDOW   30            // seconds
#define THRESHOLD_MIN_DURATION 10            // seconds
#define SLOPE_CHANGE_THRESHOLD 1.5f          // Minimum slope change ratio

// Filter Configuration
#define KALMAN_Q_DEFAULT       0.01f         // Process noise
#define KALMAN_R_DEFAULT       0.1f          // Measurement noise
#define MOVING_AVG_WINDOW      5             // Default moving average window
#define ENABLE_ADAPTIVE_FILTER 1             // Enable adaptive filtering

// BLE Configuration
#define BLE_DEVICE_NAME        "VO2Smart"
#define BLE_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_DATA_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CONTROL_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_STATUS_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define BLE_MTU_SIZE           512
#define BLE_ADVERTISING_INTERVAL 100         // ms

// WiFi Configuration
#define WIFI_DEFAULT_SSID      "VO2Smart_AP"
#define WIFI_DEFAULT_PASSWORD  "vo2smart123"
#define WIFI_AP_MODE           1             // Enable AP mode as fallback
#define WEB_SERVER_PORT        80
#define WIFI_RECONNECT_INTERVAL 30000       // ms
#define WIFI_MAX_RECONNECT_ATTEMPTS 5

// Error Handling Configuration
#define ENABLE_WATCHDOG        1
#define WATCHDOG_TIMEOUT_MS    10000
#define MAX_SENSOR_ERRORS      3
#define ERROR_RECOVERY_DELAY   5000         // ms
#define ENABLE_ERROR_LOGGING   1

// Data Validation Ranges
#define VO2_MIN_VALUE          10.0f         // ml/kg/min
#define VO2_MAX_VALUE          80.0f         // ml/kg/min
#define VCO2_MIN_VALUE         8.0f          // ml/kg/min
#define VCO2_MAX_VALUE         100.0f        // ml/kg/min
#define RER_MIN_VALUE          0.6f
#define RER_MAX_VALUE          1.3f
#define HR_MIN_VALUE           30.0f         // bpm
#define HR_MAX_VALUE           220.0f        // bpm
#define SPO2_MIN_VALUE         70.0f         // %
#define SPO2_MAX_VALUE         100.0f        // %

// Display Configuration
#define GRAPH_UPDATE_INTERVAL  100           // ms
#define SCREEN_TIMEOUT         300000        // 5 minutes
#define ENABLE_TOUCH_INTERFACE 0             // Disable if no touch screen
#define DEFAULT_BRIGHTNESS     128           // 0-255

// Power Management
#define ENABLE_POWER_SAVE      1
#define LOW_POWER_THRESHOLD    30            // Battery % for power save mode
#define SLEEP_MODE_TIMEOUT     600000        // 10 minutes of inactivity

// Debug Configuration
#define ENABLE_SERIAL_DEBUG    1
#define SERIAL_BAUD_RATE       115200
#define ENABLE_PERFORMANCE_MON 1             // Monitor task performance
#define ENABLE_MEMORY_DEBUG    0             // Memory usage debugging

// Version Information
#define FIRMWARE_VERSION       "2.1.0"
#define HARDWARE_VERSION       "1.2"
#define BUILD_DATE             __DATE__
#define BUILD_TIME             __TIME__

// Feature Flags
#define ENABLE_CALIBRATION     1             // Enable sensor calibration
#define ENABLE_DATA_LOGGING    1             // Enable data logging to flash
#define ENABLE_OTA_UPDATE      0             // Over-the-air updates
#define ENABLE_ENCRYPTION      0             // Data encryption (experimental)