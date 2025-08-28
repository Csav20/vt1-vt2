#include "DisplayManager.h"

// Static member definitions
TFT_eSPI DisplayManager::tft = TFT_eSPI();
DisplayManager::Screen DisplayManager::currentScreen = SCREEN_SPLASH;
DisplayManager::NavigationMode DisplayManager::currentMode = MODE_NORMAL;
DisplayManager::DisplayData DisplayManager::lastData = {};
unsigned long DisplayManager::lastUpdate = 0;
bool DisplayManager::needsFullRefresh = true;
DisplayManager::UpdateRegion DisplayManager::regions[9] = {};

// Initialize all regions
void initRegions() {
    for (int i = 0; i < 9; i++) {
        DisplayManager::regions[i].x = (i % 3) * DisplayManager::CELL_WIDTH;
        DisplayManager::regions[i].y = (i / 3) * DisplayManager::CELL_HEIGHT;
        DisplayManager::regions[i].width = DisplayManager::CELL_WIDTH;
        DisplayManager::regions[i].height = DisplayManager::CELL_HEIGHT;
        DisplayManager::regions[i].needsUpdate = true;
    }
}

bool DisplayManager::initialize() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    
    // Initialize update regions
    initRegions();
    
    return true;
}

void DisplayManager::setScreen(Screen screen) {
    if (screen != currentScreen) {
        currentScreen = screen;
        needsFullRefresh = true;
        // Mark all regions for update
        for (int i = 0; i < 9; i++) {
            regions[i].needsUpdate = true;
        }
    }
}

void DisplayManager::setMode(NavigationMode mode) {
    currentMode = mode;
}

void DisplayManager::update(const DisplayData& data) {
    unsigned long now = millis();
    
    // Check if data has changed significantly
    bool dataChanged = hasDataChanged(data, lastData);
    
    // Update only if needed and not too frequently (reduce flicker)
    if ((dataChanged || needsFullRefresh) && (now - lastUpdate > 50)) {
        
        // Draw current screen
        switch (currentScreen) {
            case SCREEN_SPLASH:
                drawSplashScreen();
                break;
            case SCREEN_MAIN:
                drawMainMetrics(data);
                break;
            case SCREEN_RESPIRATORY:
                drawRespiratoryData(data);
                break;
            case SCREEN_THRESHOLDS:
                drawThresholds(data);
                break;
            case SCREEN_CALORIMETRY:
                drawCalorimetry(data);
                break;
            case SCREEN_ENVIRONMENT:
                drawEnvironment(data);
                break;
            case SCREEN_PARAMETERS:
                drawParameters(data);
                break;
            case SCREEN_CONNECTIVITY:
                drawConnectivity(data);
                break;
            case SCREEN_ISO:
                drawISO(data);
                break;
            case SCREEN_DEMO_SELECTION:
                drawDemoSelection(data);
                break;
        }
        
        // Always update header
        updateHeader();
        
        lastData = data;
        lastUpdate = now;
        needsFullRefresh = false;
    }
}

void DisplayManager::forceRefresh() {
    needsFullRefresh = true;
    for (int i = 0; i < 9; i++) {
        regions[i].needsUpdate = true;
    }
}

void DisplayManager::nextScreen() {
    Screen nextScreen;
    
    switch (currentMode) {
        case MODE_DEMO:
            // Demo mode cycles through all screens
            nextScreen = (Screen)((currentScreen + 1) % SCREEN_COUNT);
            if (nextScreen == SCREEN_SPLASH) nextScreen = SCREEN_MAIN;
            break;
            
        case MODE_VO2:
            // VO2 mode only shows relevant screens
            switch (currentScreen) {
                case SCREEN_MAIN:
                    nextScreen = SCREEN_RESPIRATORY;
                    break;
                case SCREEN_RESPIRATORY:
                    nextScreen = SCREEN_THRESHOLDS;
                    break;
                case SCREEN_THRESHOLDS:
                    nextScreen = SCREEN_CALORIMETRY;
                    break;
                default:
                    nextScreen = SCREEN_MAIN;
                    break;
            }
            break;
            
        case MODE_NORMAL:
        default:
            // Normal mode cycles through main screens
            switch (currentScreen) {
                case SCREEN_MAIN:
                    nextScreen = SCREEN_RESPIRATORY;
                    break;
                case SCREEN_RESPIRATORY:
                    nextScreen = SCREEN_ENVIRONMENT;
                    break;
                case SCREEN_ENVIRONMENT:
                    nextScreen = SCREEN_PARAMETERS;
                    break;
                case SCREEN_PARAMETERS:
                    nextScreen = SCREEN_CONNECTIVITY;
                    break;
                default:
                    nextScreen = SCREEN_MAIN;
                    break;
            }
            break;
    }
    
    setScreen(nextScreen);
}

void DisplayManager::previousScreen() {
    // Similar logic but in reverse
    Screen prevScreen;
    
    switch (currentMode) {
        case MODE_DEMO:
            prevScreen = (Screen)((currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
            if (prevScreen == SCREEN_SPLASH) prevScreen = SCREEN_ISO;
            break;
            
        case MODE_VO2:
            switch (currentScreen) {
                case SCREEN_MAIN:
                    prevScreen = SCREEN_CALORIMETRY;
                    break;
                case SCREEN_RESPIRATORY:
                    prevScreen = SCREEN_MAIN;
                    break;
                case SCREEN_THRESHOLDS:
                    prevScreen = SCREEN_RESPIRATORY;
                    break;
                case SCREEN_CALORIMETRY:
                    prevScreen = SCREEN_THRESHOLDS;
                    break;
                default:
                    prevScreen = SCREEN_MAIN;
                    break;
            }
            break;
            
        case MODE_NORMAL:
        default:
            switch (currentScreen) {
                case SCREEN_MAIN:
                    prevScreen = SCREEN_CONNECTIVITY;
                    break;
                case SCREEN_RESPIRATORY:
                    prevScreen = SCREEN_MAIN;
                    break;
                case SCREEN_ENVIRONMENT:
                    prevScreen = SCREEN_RESPIRATORY;
                    break;
                case SCREEN_PARAMETERS:
                    prevScreen = SCREEN_ENVIRONMENT;
                    break;
                case SCREEN_CONNECTIVITY:
                    prevScreen = SCREEN_PARAMETERS;
                    break;
                default:
                    prevScreen = SCREEN_MAIN;
                    break;
            }
            break;
    }
    
    setScreen(prevScreen);
}

DisplayManager::Screen DisplayManager::getCurrentScreen() {
    return currentScreen;
}

DisplayManager::NavigationMode DisplayManager::getCurrentMode() {
    return currentMode;
}

void DisplayManager::drawHeader(const DisplayData& data) {
    // Header area (top 20 pixels)
    tft.fillRect(0, 0, 240, 20, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.drawString("VO2Smart", 5, 2);
    
    // Time
    tft.drawString(data.time, 80, 2);
    
    // Connectivity status
    if (data.bleConnected) {
        tft.setTextColor(TFT_GREEN, TFT_NAVY);
        tft.drawString("BLE", 150, 2);
    }
    if (data.wifiConnected) {
        tft.setTextColor(TFT_GREEN, TFT_NAVY);
        tft.drawString("WiFi", 180, 2);
    }
    
    // Battery
    drawBatteryIndicator(data.batteryVoltage);
}

void DisplayManager::drawMainMetrics(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    // VO2 (cell 0)
    updateCell(0, String(data.vo2, 1), "mL/min");
    
    // VCO2 (cell 1)
    updateCell(1, String(data.vco2, 1), "mL/min");
    
    // RER (cell 2)
    updateCell(2, String(data.rer, 2), "");
    
    // O2% (cell 3)
    updateCell(3, String(data.o2Percent, 1), "%");
    
    // CO2 ppm (cell 4)
    updateCell(4, String(data.co2Ppm, 0), "ppm");
    
    // Pressure (cell 5)
    updateCell(5, String(data.pressure, 1), "Pa");
    
    // Temperature (cell 6)
    updateCell(6, String(data.temperature, 1), "°C");
    
    // Humidity (cell 7)
    updateCell(7, String(data.humidity, 0), "%");
    
    // Mode indicator (cell 8)
    String modeStr = currentMode == MODE_DEMO ? "DEMO" : 
                     currentMode == MODE_VO2 ? "VO2" : "NORM";
    updateCell(8, modeStr, "");
}

void DisplayManager::drawRespiratoryData(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    // Respiratory specific display
    updateCell(0, "VE", String(data.pressure * 0.1, 1) + " L/min");
    updateCell(1, "TV", "500 mL");
    updateCell(2, "RR", "15 /min");
    updateCell(3, String(data.o2Percent, 1), "% O2");
    updateCell(4, String(data.co2Ppm, 0), "CO2 ppm");
    updateCell(5, "I:E", "1:2");
    updateCell(6, String(data.temperature, 1), "°C");
    updateCell(7, String(data.humidity, 0), "% RH");
    updateCell(8, "RESP", "");
}

void DisplayManager::drawThresholds(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    updateCell(0, "VT1", "-- W");
    updateCell(1, "VT2", "-- W");
    updateCell(2, "VO2max", "-- mL");
    updateCell(3, "HR", "-- bpm");
    updateCell(4, "Power", "-- W");
    updateCell(5, "Lactate", "-- mmol");
    updateCell(6, "Status", "Ready");
    updateCell(7, "Time", "00:00");
    updateCell(8, "THRESH", "");
}

void DisplayManager::drawCalorimetry(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    float calories = data.vo2 * 0.005; // Rough calculation
    updateCell(0, String(calories, 1), "kcal/min");
    updateCell(1, "CHO", "50%");
    updateCell(2, "FAT", "50%");
    updateCell(3, String(data.rer, 2), "RER");
    updateCell(4, "EE", String(calories * 60, 0) + " kcal/h");
    updateCell(5, "Substrat", "Mixed");
    updateCell(6, "Efficiency", "22%");
    updateCell(7, "METs", String(data.vo2 / 70.0 / 3.5, 1));
    updateCell(8, "METAB", "");
}

void DisplayManager::drawEnvironment(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    updateCell(0, String(data.temperature, 1), "°C");
    updateCell(1, String(data.humidity, 0), "% RH");
    updateCell(2, String(data.pressure * 0.01, 1), "hPa");
    updateCell(3, "Altitude", "100 m");
    updateCell(4, "Density", "1.2 kg/m3");
    updateCell(5, "BTPS", "1.10");
    updateCell(6, "STPD", "0.86");
    updateCell(7, "Ambient", "OK");
    updateCell(8, "ENV", "");
}

void DisplayManager::drawParameters(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    updateCell(0, "Weight", "70 kg");
    updateCell(1, "Method", "V-Slope");
    updateCell(2, "O2 Cal", "1.00");
    updateCell(3, "P Cal", "0.0");
    updateCell(4, "Flow Cal", "1.00");
    updateCell(5, "Filter", "ON");
    updateCell(6, "Auto Cal", "OFF");
    updateCell(7, "Save", "Ready");
    updateCell(8, "PARAMS", "");
}

void DisplayManager::drawConnectivity(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    updateCell(0, data.bleConnected ? "Connected" : "Advertising", "BLE");
    updateCell(1, data.wifiConnected ? "Connected" : "Disconnected", "WiFi");
    updateCell(2, "Data Rate", "1 Hz");
    updateCell(3, "Protocol", "JSON");
    updateCell(4, "Clients", data.bleConnected ? "1" : "0");
    updateCell(5, "Signal", "-60 dBm");
    updateCell(6, "Packets", "1234");
    updateCell(7, "Errors", "0");
    updateCell(8, "CONN", "");
}

void DisplayManager::drawISO(const DisplayData& data) {
    if (needsFullRefresh) {
        drawGrid();
    }
    
    updateCell(0, "ISO 5167", "Flow");
    updateCell(1, "ISO 8996", "Metab");
    updateCell(2, "ATS/ERS", "Spirom");
    updateCell(3, "Accuracy", "±2%");
    updateCell(4, "Precision", "±1%");
    updateCell(5, "Drift", "<0.1%/h");
    updateCell(6, "Response", "<100ms");
    updateCell(7, "Cal Freq", "Daily");
    updateCell(8, "STANDARD", "");
}

void DisplayManager::drawDemoSelection(const DisplayData& data) {
    if (needsFullRefresh) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.drawString("Select Mode:", 80, 50, 2);
        tft.drawString("1. Normal Mode", 60, 80, 2);
        tft.drawString("2. Demo Mode", 60, 110, 2);
        tft.drawString("3. VO2 Mode", 60, 140, 2);
    }
}

void DisplayManager::drawSplashScreen() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_CYAN);
    tft.drawString("VO2Smart", 80, 60, 4);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Version 3.1", 80, 100, 2);
    tft.drawString("Initializing...", 70, 130, 2);
}

void DisplayManager::updateCell(int cellIndex, const String& value, const String& unit) {
    if (cellIndex < 0 || cellIndex >= 9) return;
    
    UpdateRegion& region = regions[cellIndex];
    
    // Only update if region needs updating
    if (region.needsUpdate || needsFullRefresh) {
        // Clear cell area
        tft.fillRect(region.x + 1, region.y + 21, region.width - 2, region.height - 2, TFT_BLACK);
        
        // Draw value
        tft.setTextColor(TFT_WHITE);
        tft.drawString(value, region.x + 5, region.y + 25, 2);
        
        // Draw unit if provided
        if (unit.length() > 0) {
            tft.setTextColor(TFT_YELLOW);
            tft.drawString(unit, region.x + 5, region.y + 45, 1);
        }
        
        region.needsUpdate = false;
    }
}

void DisplayManager::updateHeader(bool forceUpdate) {
    // Header is updated every time for time and connectivity
    // No optimization here as it changes frequently
}

void DisplayManager::markRegionForUpdate(int cellIndex) {
    if (cellIndex >= 0 && cellIndex < 9) {
        regions[cellIndex].needsUpdate = true;
    }
}

bool DisplayManager::hasDataChanged(const DisplayData& newData, const DisplayData& oldData) {
    // Simple threshold-based change detection to avoid updating for minor variations
    const float THRESHOLD = 0.1;
    
    return (abs(newData.vo2 - oldData.vo2) > THRESHOLD ||
            abs(newData.vco2 - oldData.vco2) > 0.05 ||
            abs(newData.o2Percent - oldData.o2Percent) > 0.1 ||
            abs(newData.co2Ppm - oldData.co2Ppm) > 10 ||
            abs(newData.pressure - oldData.pressure) > 1.0 ||
            abs(newData.temperature - oldData.temperature) > 0.5 ||
            newData.bleConnected != oldData.bleConnected ||
            newData.wifiConnected != oldData.wifiConnected ||
            newData.mode != oldData.mode);
}

void DisplayManager::drawGrid() {
    // Draw grid layout for 3x3 cells
    tft.fillRect(0, 20, 240, 300, TFT_BLACK);
    
    // Draw grid lines
    tft.drawRect(0, 20, 240, 300, TFT_WHITE);
    
    // Vertical lines
    for (int i = 1; i < GRID_COLS; i++) {
        tft.drawLine(i * CELL_WIDTH, 20, i * CELL_WIDTH, 320, TFT_WHITE);
    }
    
    // Horizontal lines  
    for (int i = 1; i < GRID_ROWS; i++) {
        tft.drawLine(0, 20 + i * CELL_HEIGHT, 240, 20 + i * CELL_HEIGHT, TFT_WHITE);
    }
}

void DisplayManager::drawBatteryIndicator(float voltage) {
    // Draw battery indicator in top right
    int x = 210;
    int y = 2;
    int width = 25;
    int height = 15;
    
    // Battery outline
    tft.drawRect(x, y, width, height, TFT_WHITE);
    tft.drawRect(x + width, y + 3, 3, height - 6, TFT_WHITE);
    
    // Battery level (3.0V = empty, 4.2V = full)
    float level = (voltage - 3.0) / 1.2;
    level = constrain(level, 0.0, 1.0);
    
    int fillWidth = (int)(level * (width - 2));
    uint16_t color = level > 0.3 ? TFT_GREEN : TFT_RED;
    
    if (fillWidth > 0) {
        tft.fillRect(x + 1, y + 1, fillWidth, height - 2, color);
    }
}

void DisplayManager::drawConnectivityIcons(bool ble, bool wifi) {
    // Draw BLE icon
    if (ble) {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("B", 150, 2);
    }
    
    // Draw WiFi icon
    if (wifi) {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("W", 170, 2);
    }
}