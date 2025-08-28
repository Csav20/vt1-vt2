#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "MetabolicCalculator.h"
#include "SensorManager.h"
#include "utils/GraphRenderer.h"

/**
 * @brief Advanced display management with dynamic graphs and real-time visualization
 * 
 * Features:
 * - Dynamic real-time graphs for VO2, VCO2, and thresholds
 * - Multiple screen layouts optimized for different data types
 * - Efficient partial screen updates to reduce flicker
 * - User-friendly error messages and status indicators
 */
class DisplayManager {
public:
    enum Screen {
        MAIN_METRICS,           // Primary VO2/VCO2 display with live graph
        RESPIRATORY_DETAILED,   // Detailed respiratory parameters
        THRESHOLDS_ANALYSIS,    // VT1/VT2 threshold analysis with trends
        CALORIMETRY,           // Caloric expenditure and substrate utilization
        ENVIRONMENT_SENSORS,    // Environmental and sensor status
        ERROR_DISPLAY,         // Error messages and diagnostics
        CALIBRATION_MODE       // Calibration interface
    };

    enum GraphType {
        GRAPH_VO2_REALTIME,    // Real-time VO2 with 60-second window
        GRAPH_VCO2_REALTIME,   // Real-time VCO2 with 60-second window
        GRAPH_RER_TREND,       // RER trend analysis
        GRAPH_THRESHOLD_EVOLUTION, // VT1/VT2 evolution over time
        GRAPH_HEART_RATE,      // Heart rate variability
        GRAPH_EFFICIENCY       // Metabolic efficiency over time
    };

    struct DisplayConfig {
        uint16_t backgroundColor;
        uint16_t primaryTextColor;
        uint16_t secondaryTextColor;
        uint16_t graphLineColor;
        uint16_t gridColor;
        uint16_t errorColor;
        uint16_t warningColor;
        uint16_t successColor;
        uint8_t brightness;
        bool autoRotate;
        bool showGrid;
        bool enableAnimations;
    };

    // Core display functions
    static void initialize();
    static void setScreen(Screen screen);
    static void update();
    static void forceRefresh();
    static void shutdown();
    
    // Configuration and customization
    static void setConfig(const DisplayConfig& config);
    static void setBrightness(uint8_t brightness);
    static void setOrientation(uint8_t rotation);
    
    // Dynamic graph management
    static void updateGraph(GraphType type, float newValue);
    static void clearGraph(GraphType type);
    static void setGraphTimeWindow(GraphType type, uint32_t windowSeconds);
    static void setGraphScale(GraphType type, float minValue, float maxValue);
    
    // Status and error display
    static void showError(const char* title, const char* message, uint32_t duration = 5000);
    static void showWarning(const char* message, uint32_t duration = 3000);
    static void showSuccess(const char* message, uint32_t duration = 2000);
    static void clearStatusMessage();
    
    // Calibration interface
    static void showCalibrationProgress(const char* sensorName, float progress);
    static void showCalibrationResult(bool success, const char* message);
    static void updateCalibrationStep(int step, int totalSteps, const char* instruction);
    
    // Touch interface (if available)
    static bool handleTouch(uint16_t x, uint16_t y);
    static void enableTouchInterface(bool enable);
    
    // Performance optimization
    static void enablePartialUpdates(bool enable);
    static void setUpdateFrequency(uint8_t frequencyHz);
    static uint32_t getFrameRate();
    static uint32_t getMemoryUsage();

private:
    static TFT_eSPI tft;
    static Screen currentScreen;
    static Screen previousScreen;
    static DisplayConfig config;
    
    // Graph renderers for different data types
    static GraphRenderer vo2Graph;
    static GraphRenderer vco2Graph;
    static GraphRenderer rerGraph;
    static GraphRenderer thresholdGraph;
    static GraphRenderer hrGraph;
    static GraphRenderer efficiencyGraph;
    
    // Display state management
    static bool needsRefresh;
    static bool partialUpdatesEnabled;
    static uint32_t lastUpdate;
    static uint32_t updateInterval;
    static uint32_t frameCounter;
    static uint32_t lastFrameTime;
    
    // Status message system
    static bool statusMessageActive;
    static char statusMessage[128];
    static uint16_t statusColor;
    static uint32_t statusExpiry;
    
    // Touch interface state
    static bool touchEnabled;
    static uint16_t lastTouchX;
    static uint16_t lastTouchY;
    static uint32_t lastTouchTime;
    
    // Screen-specific drawing functions
    static void drawMainMetrics();
    static void drawRespiratoryDetailed();
    static void drawThresholdsAnalysis();
    static void drawCalorimetry();
    static void drawEnvironmentSensors();
    static void drawErrorDisplay();
    static void drawCalibrationMode();
    
    // UI components
    static void drawHeader(const char* title);
    static void drawBatteryIndicator(float voltage, bool charging = false);
    static void drawConnectionStatus(bool bleConnected, bool wifiConnected);
    static void drawSensorStatus(int sensorMask);
    static void drawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, 
                               float progress, uint16_t color);
    static void drawButton(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                          const char* text, bool pressed = false);
    
    // Data visualization helpers
    static void drawValueBox(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                           const char* label, float value, const char* unit, 
                           uint16_t textColor = 0xFFFF);
    static void drawTrendIndicator(uint16_t x, uint16_t y, float currentValue, 
                                 float previousValue, bool showArrow = true);
    static void drawQualityIndicator(uint16_t x, uint16_t y, float quality);
    
    // Graph-specific functions
    static void updateRealTimeGraphs();
    static void drawThresholdMarkers();
    static void drawZoneIndicators(float vo2Max);
    static void animateThresholdDetection(bool vt1Detected, bool vt2Detected);
    
    // Layout management
    static void calculateLayout();
    static void optimizeForOrientation(uint8_t rotation);
    
    // Memory and performance optimization
    static void clearScreenBuffer();
    static void optimizeRefreshRate();
    static bool shouldUpdateRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    
    // Constants for layout optimization
    static const uint16_t SCREEN_WIDTH = 240;
    static const uint16_t SCREEN_HEIGHT = 320;
    static const uint8_t HEADER_HEIGHT = 30;
    static const uint8_t FOOTER_HEIGHT = 20;
    static const uint8_t MARGIN = 5;
    static const uint8_t GRAPH_HEIGHT = 80;
    static const uint8_t VALUE_BOX_HEIGHT = 50;
    
    // Color constants
    static const uint16_t COLOR_BACKGROUND = 0x0000;    // Black
    static const uint16_t COLOR_PRIMARY = 0xFFFF;       // White
    static const uint16_t COLOR_SECONDARY = 0x8410;     // Gray
    static const uint16_t COLOR_VO2 = 0x07E0;          // Green
    static const uint16_t COLOR_VCO2 = 0xF800;         // Red
    static const uint16_t COLOR_RER = 0x001F;          // Blue
    static const uint16_t COLOR_VT1 = 0xFFE0;          // Yellow
    static const uint16_t COLOR_VT2 = 0xF81F;          // Magenta
    static const uint16_t COLOR_ERROR = 0xF800;        // Red
    static const uint16_t COLOR_WARNING = 0xFFE0;      // Yellow
    static const uint16_t COLOR_SUCCESS = 0x07E0;      // Green
};