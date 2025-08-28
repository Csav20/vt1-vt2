#include "DisplayManager.h"
#include "config.h"
#include <Arduino.h>

// Static member definitions
TFT_eSPI DisplayManager::tft = TFT_eSPI();
DisplayManager::Screen DisplayManager::currentScreen = MAIN_METRICS;
DisplayManager::Screen DisplayManager::previousScreen = MAIN_METRICS;
DisplayManager::DisplayConfig DisplayManager::config;
GraphRenderer DisplayManager::vo2Graph;
GraphRenderer DisplayManager::vco2Graph;
GraphRenderer DisplayManager::rerGraph;
GraphRenderer DisplayManager::thresholdGraph;
GraphRenderer DisplayManager::hrGraph;
GraphRenderer DisplayManager::efficiencyGraph;
bool DisplayManager::needsRefresh = true;
bool DisplayManager::partialUpdatesEnabled = true;
uint32_t DisplayManager::lastUpdate = 0;
uint32_t DisplayManager::updateInterval = 1000 / UI_TASK_FREQ_HZ;
uint32_t DisplayManager::frameCounter = 0;
uint32_t DisplayManager::lastFrameTime = 0;
bool DisplayManager::statusMessageActive = false;
char DisplayManager::statusMessage[128] = "";
uint16_t DisplayManager::statusColor = COLOR_PRIMARY;
uint32_t DisplayManager::statusExpiry = 0;
bool DisplayManager::touchEnabled = false;
uint16_t DisplayManager::lastTouchX = 0;
uint16_t DisplayManager::lastTouchY = 0;
uint32_t DisplayManager::lastTouchTime = 0;

void DisplayManager::initialize() {
    Serial.println("Initializing display...");
    
    // Initialize TFT display
    tft.init();
    tft.setRotation(DISPLAY_ROTATION);
    tft.fillScreen(COLOR_BACKGROUND);
    
    // Set default configuration
    config.backgroundColor = COLOR_BACKGROUND;
    config.primaryTextColor = COLOR_PRIMARY;
    config.secondaryTextColor = COLOR_SECONDARY;
    config.graphLineColor = COLOR_VO2;
    config.gridColor = COLOR_SECONDARY;
    config.errorColor = COLOR_ERROR;
    config.warningColor = COLOR_WARNING;
    config.successColor = COLOR_SUCCESS;
    config.brightness = DEFAULT_BRIGHTNESS;
    config.autoRotate = false;
    config.showGrid = true;
    config.enableAnimations = true;
    
    // Initialize graph renderers
    GraphRenderer::GraphConfig graphConfig;
    graphConfig.x = 10;
    graphConfig.y = 60;
    graphConfig.width = SCREEN_WIDTH - 20;
    graphConfig.height = GRAPH_HEIGHT;
    graphConfig.lineColor = COLOR_VO2;
    graphConfig.backgroundColor = COLOR_BACKGROUND;
    graphConfig.gridColor = COLOR_SECONDARY;
    graphConfig.showGrid = true;
    graphConfig.type = GraphRenderer::LINE_GRAPH;
    
    vo2Graph.initialize(&tft, graphConfig);
    
    graphConfig.lineColor = COLOR_VCO2;
    vco2Graph.initialize(&tft, graphConfig);
    
    // Show initialization screen
    drawHeader("VO2Smart v" FIRMWARE_VERSION);
    tft.setTextColor(COLOR_PRIMARY);
    tft.setTextSize(2);
    tft.drawString("Initializing...", 50, 150);
    
    Serial.println("Display initialized");
}

void DisplayManager::update() {
    uint32_t currentTime = millis();
    
    // Check if update is needed
    if (currentTime - lastUpdate < updateInterval && !needsRefresh) {
        return;
    }
    
    // Clear status message if expired
    if (statusMessageActive && currentTime > statusExpiry) {
        clearStatusMessage();
    }
    
    // Update frame counter
    frameCounter++;
    if (currentTime - lastFrameTime >= 1000) {
        // Could log FPS here if needed
        lastFrameTime = currentTime;
        frameCounter = 0;
    }
    
    // Render current screen
    switch (currentScreen) {
        case MAIN_METRICS:
            drawMainMetrics();
            break;
        case RESPIRATORY_DETAILED:
            drawRespiratoryDetailed();
            break;
        case THRESHOLDS_ANALYSIS:
            drawThresholdsAnalysis();
            break;
        case CALORIMETRY:
            drawCalorimetry();
            break;
        case ENVIRONMENT_SENSORS:
            drawEnvironmentSensors();
            break;
        case ERROR_DISPLAY:
            drawErrorDisplay();
            break;
        case CALIBRATION_MODE:
            drawCalibrationMode();
            break;
    }
    
    // Update graphs if enabled
    updateRealTimeGraphs();
    
    // Draw status message if active
    if (statusMessageActive) {
        tft.setTextColor(statusColor);
        tft.setTextSize(1);
        tft.drawString(statusMessage, 10, SCREEN_HEIGHT - 20);
    }
    
    lastUpdate = currentTime;
    needsRefresh = false;
}

void DisplayManager::setScreen(Screen screen) {
    if (screen != currentScreen) {
        previousScreen = currentScreen;
        currentScreen = screen;
        needsRefresh = true;
        
        Serial.printf("Screen changed to: %d\n", screen);
    }
}

void DisplayManager::drawMainMetrics() {
    if (!needsRefresh && partialUpdatesEnabled) return;
    
    // Clear screen
    tft.fillScreen(config.backgroundColor);
    
    // Draw header
    drawHeader("VO2/VCO2 Monitor");
    
    // Draw main metrics in grid layout
    drawValueBox(10, 40, 100, 50, "VO2", 42.5f, "ml/kg/min", COLOR_VO2);
    drawValueBox(120, 40, 100, 50, "VCO2", 38.1f, "ml/kg/min", COLOR_VCO2);
    
    drawValueBox(10, 100, 100, 50, "RER", 0.89f, "", COLOR_RER);
    drawValueBox(120, 100, 100, 50, "HR", 145.0f, "bpm", COLOR_PRIMARY);
    
    // Draw real-time graph area
    tft.drawRect(10, 160, SCREEN_WIDTH - 20, GRAPH_HEIGHT, config.gridColor);
    tft.setTextColor(config.secondaryTextColor);
    tft.setTextSize(1);
    tft.drawString("Real-time VO2/VCO2", 15, 165);
    
    // Draw threshold indicators
    drawValueBox(10, 250, 100, 30, "VT1", 35.2f, "ml/kg/min", COLOR_VT1);
    drawValueBox(120, 250, 100, 30, "VT2", 44.1f, "ml/kg/min", COLOR_VT2);
    
    // Draw system status
    drawConnectionStatus(true, false); // Example: BLE connected, WiFi disconnected
    drawBatteryIndicator(3.7f, false);
}

void DisplayManager::drawRespiratoryDetailed() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("Respiratory Analysis");
    
    // Detailed respiratory parameters
    drawValueBox(10, 40, 220, 30, "Ventilation Rate", 28.4f, "L/min", COLOR_PRIMARY);
    drawValueBox(10, 80, 110, 30, "Tidal Vol", 580.0f, "ml", COLOR_SECONDARY);
    drawValueBox(120, 80, 110, 30, "Resp Rate", 15.0f, "br/min", COLOR_SECONDARY);
    
    // Placeholder for additional respiratory metrics
    tft.setTextColor(config.secondaryTextColor);
    tft.setTextSize(1);
    tft.drawString("Additional respiratory analysis", 10, 130);
    tft.drawString("would be displayed here", 10, 145);
}

void DisplayManager::drawThresholdsAnalysis() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("Threshold Analysis");
    
    // VT1/VT2 analysis display
    drawValueBox(10, 40, 220, 40, "First Ventilatory Threshold", 35.2f, "ml/kg/min", COLOR_VT1);
    drawValueBox(10, 90, 220, 40, "Second Ventilatory Threshold", 44.1f, "ml/kg/min", COLOR_VT2);
    
    // Threshold detection status
    tft.setTextColor(COLOR_SUCCESS);
    tft.setTextSize(1);
    tft.drawString("VT1 Detected: YES", 10, 140);
    tft.setTextColor(COLOR_WARNING);
    tft.drawString("VT2 Detected: PENDING", 10, 155);
    
    // Progress indicators
    drawProgressBar(10, 170, 220, 10, 0.75f, COLOR_VT1);
    tft.setTextColor(config.secondaryTextColor);
    tft.drawString("Analysis Progress", 10, 185);
}

void DisplayManager::drawCalorimetry() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("Calorimetry");
    
    // Energy expenditure and substrate utilization
    drawValueBox(10, 40, 110, 40, "Energy", 12.5f, "kcal/min", COLOR_PRIMARY);
    drawValueBox(120, 40, 110, 40, "Total", 156.0f, "kcal", COLOR_SECONDARY);
    
    drawValueBox(10, 90, 110, 40, "Carbs", 65.0f, "%", COLOR_SUCCESS);
    drawValueBox(120, 90, 110, 40, "Fats", 35.0f, "%", COLOR_WARNING);
    
    // Metabolic efficiency
    tft.setTextColor(config.primaryTextColor);
    tft.setTextSize(1);
    tft.drawString("Metabolic Efficiency: Good", 10, 140);
}

void DisplayManager::drawEnvironmentSensors() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("Environment & Sensors");
    
    // Environmental conditions
    drawValueBox(10, 40, 110, 30, "Temp", 23.5f, "°C", COLOR_PRIMARY);
    drawValueBox(120, 40, 110, 30, "Press", 1013.2f, "hPa", COLOR_SECONDARY);
    
    drawValueBox(10, 80, 110, 30, "Humid", 65.0f, "%", COLOR_SECONDARY);
    drawValueBox(120, 80, 110, 30, "Altitude", 150.0f, "m", COLOR_SECONDARY);
    
    // Sensor status indicators
    drawSensorStatus(0xFF); // All sensors OK
}

void DisplayManager::drawErrorDisplay() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("System Error");
    
    tft.setTextColor(config.errorColor);
    tft.setTextSize(2);
    tft.drawString("ERROR", 80, 80);
    
    tft.setTextColor(config.primaryTextColor);
    tft.setTextSize(1);
    tft.drawString("Check sensor connections", 50, 120);
    tft.drawString("Press RESET to continue", 50, 140);
}

void DisplayManager::drawCalibrationMode() {
    tft.fillScreen(config.backgroundColor);
    drawHeader("Calibration Mode");
    
    tft.setTextColor(config.warningColor);
    tft.setTextSize(1);
    tft.drawString("Sensor calibration in progress", 30, 80);
    
    // Example calibration progress
    drawProgressBar(30, 100, 180, 15, 0.45f, config.warningColor);
    tft.setTextColor(config.primaryTextColor);
    tft.drawString("O2 Sensor: 45%", 30, 120);
}

void DisplayManager::drawHeader(const char* title) {
    tft.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, config.primaryTextColor);
    tft.setTextColor(config.backgroundColor);
    tft.setTextSize(1);
    tft.drawString(title, 5, 10);
    
    // Draw time (placeholder)
    tft.drawString("14:30", SCREEN_WIDTH - 35, 10);
}

void DisplayManager::drawValueBox(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                 const char* label, float value, const char* unit, 
                                 uint16_t textColor) {
    // Draw border
    tft.drawRect(x, y, width, height, config.gridColor);
    
    // Draw label
    tft.setTextColor(config.secondaryTextColor);
    tft.setTextSize(1);
    tft.drawString(label, x + 5, y + 5);
    
    // Draw value
    tft.setTextColor(textColor);
    tft.setTextSize(2);
    char valueStr[32];
    snprintf(valueStr, sizeof(valueStr), "%.1f", value);
    tft.drawString(valueStr, x + 5, y + 15);
    
    // Draw unit
    tft.setTextColor(config.secondaryTextColor);
    tft.setTextSize(1);
    tft.drawString(unit, x + 5, y + height - 15);
}

void DisplayManager::drawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, 
                                   float progress, uint16_t color) {
    // Draw border
    tft.drawRect(x, y, width, height, config.gridColor);
    
    // Draw progress fill
    uint16_t fillWidth = (uint16_t)(width * constrain(progress, 0.0f, 1.0f));
    tft.fillRect(x + 1, y + 1, fillWidth - 1, height - 2, color);
}

void DisplayManager::drawConnectionStatus(bool bleConnected, bool wifiConnected) {
    uint16_t x = SCREEN_WIDTH - 50;
    uint16_t y = HEADER_HEIGHT + 5;
    
    // BLE status
    tft.setTextColor(bleConnected ? COLOR_SUCCESS : COLOR_ERROR);
    tft.setTextSize(1);
    tft.drawString("BLE", x, y);
    
    // WiFi status
    tft.setTextColor(wifiConnected ? COLOR_SUCCESS : COLOR_ERROR);
    tft.drawString("WiFi", x, y + 12);
}

void DisplayManager::drawBatteryIndicator(float voltage, bool charging) {
    uint16_t x = SCREEN_WIDTH - 30;
    uint16_t y = HEADER_HEIGHT + 5;
    
    // Simple battery indicator
    uint8_t percentage = (uint8_t)((voltage - 3.0f) / 1.2f * 100.0f); // Rough LiPo calculation
    percentage = constrain(percentage, 0, 100);
    
    uint16_t color = percentage > 20 ? COLOR_SUCCESS : COLOR_ERROR;
    if (charging) color = COLOR_WARNING;
    
    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.drawString(String(percentage) + "%", x, y);
}

void DisplayManager::drawSensorStatus(int sensorMask) {
    uint16_t y = 120;
    
    tft.setTextSize(1);
    tft.setTextColor(config.secondaryTextColor);
    tft.drawString("Sensors:", 10, y);
    
    const char* sensorNames[] = {"HR", "NIRS", "ToF", "Temp", "Env", "IMU"};
    
    for (int i = 0; i < 6; i++) {
        bool sensorOK = (sensorMask & (1 << i)) != 0;
        tft.setTextColor(sensorOK ? COLOR_SUCCESS : COLOR_ERROR);
        tft.drawString(sensorNames[i], 10 + i * 35, y + 12);
    }
}

void DisplayManager::updateRealTimeGraphs() {
    // Update graphs with new data points
    // This would typically receive data from the metabolic calculator
    static uint32_t lastGraphUpdate = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastGraphUpdate > 1000) { // Update every second
        // Example: add random data points for demonstration
        float vo2Value = 40.0f + random(-50, 50) / 10.0f;
        float vco2Value = vo2Value * 0.85f + random(-20, 20) / 10.0f;
        
        vo2Graph.addDataPoint(vo2Value);
        vco2Graph.addDataPoint(vco2Value);
        
        lastGraphUpdate = currentTime;
    }
    
    // Render graphs if on appropriate screen
    if (currentScreen == MAIN_METRICS) {
        vo2Graph.render();
    }
}

void DisplayManager::showError(const char* title, const char* message, uint32_t duration) {
    strncpy(statusMessage, message, sizeof(statusMessage) - 1);
    statusMessage[sizeof(statusMessage) - 1] = '\0';
    statusColor = config.errorColor;
    statusMessageActive = true;
    statusExpiry = duration > 0 ? millis() + duration : UINT32_MAX;
    needsRefresh = true;
    
    Serial.printf("Display Error: %s - %s\n", title, message);
}

void DisplayManager::showWarning(const char* message, uint32_t duration) {
    strncpy(statusMessage, message, sizeof(statusMessage) - 1);
    statusMessage[sizeof(statusMessage) - 1] = '\0';
    statusColor = config.warningColor;
    statusMessageActive = true;
    statusExpiry = duration > 0 ? millis() + duration : UINT32_MAX;
    needsRefresh = true;
    
    Serial.printf("Display Warning: %s\n", message);
}

void DisplayManager::showSuccess(const char* message, uint32_t duration) {
    strncpy(statusMessage, message, sizeof(statusMessage) - 1);
    statusMessage[sizeof(statusMessage) - 1] = '\0';
    statusColor = config.successColor;
    statusMessageActive = true;
    statusExpiry = duration > 0 ? millis() + duration : UINT32_MAX;
    needsRefresh = true;
    
    Serial.printf("Display Success: %s\n", message);
}

void DisplayManager::clearStatusMessage() {
    statusMessageActive = false;
    statusMessage[0] = '\0';
    needsRefresh = true;
}

void DisplayManager::shutdown() {
    Serial.println("Shutting down display...");
    tft.fillScreen(COLOR_BACKGROUND);
    // Additional cleanup if needed
}

// Stub implementations for other methods
void DisplayManager::setBrightness(uint8_t brightness) {
    config.brightness = brightness;
    // Implement brightness control
}

void DisplayManager::forceRefresh() {
    needsRefresh = true;
}

bool DisplayManager::handleTouch(uint16_t x, uint16_t y) {
    if (!touchEnabled) return false;
    
    lastTouchX = x;
    lastTouchY = y;
    lastTouchTime = millis();
    
    // Handle touch interactions based on current screen
    // This would implement touch-based navigation
    
    return true;
}

void DisplayManager::updateGraph(GraphType type, float newValue) {
    switch (type) {
        case GRAPH_VO2_REALTIME:
            vo2Graph.addDataPoint(newValue);
            break;
        case GRAPH_VCO2_REALTIME:
            vco2Graph.addDataPoint(newValue);
            break;
        // Add other graph types as needed
    }
}