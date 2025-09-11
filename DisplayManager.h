#pragma once
#include <TFT_eSPI.h>

/**
 * Display Manager - Optimized screen rendering to avoid flickering
 * Updates only changed elements instead of full screen redraws
 */
class DisplayManager {
public:
    enum Screen {
        SCREEN_SPLASH = 0,
        SCREEN_MAIN = 1,
        SCREEN_RESPIRATORY = 2, 
        SCREEN_THRESHOLDS = 3,
        SCREEN_CALORIMETRY = 4,
        SCREEN_ENVIRONMENT = 5,
        SCREEN_PARAMETERS = 6,
        SCREEN_CONNECTIVITY = 7,
        SCREEN_ISO = 8,
        SCREEN_DEMO_SELECTION = 9,
        SCREEN_COUNT = 10
    };
    
    enum NavigationMode {
        MODE_NORMAL,
        MODE_DEMO,
        MODE_VO2
    };
    
    struct DisplayData {
        float vo2;
        float vco2;
        float rer;
        float o2Percent;
        float co2Ppm;
        float pressure;
        float temperature;
        float humidity;
        float batteryVoltage;
        bool bleConnected;
        bool wifiConnected;
        String time;
        NavigationMode mode;
    };
    
private:
    static TFT_eSPI tft;
    static Screen currentScreen;
    static NavigationMode currentMode;
    static DisplayData lastData;
    static unsigned long lastUpdate;
    static bool needsFullRefresh;
    
    // Layout constants
    static const int GRID_COLS = 3;
    static const int GRID_ROWS = 3;
    static const int CELL_WIDTH = 80;
    static const int CELL_HEIGHT = 60;
    
    // Update regions for optimized rendering
    struct UpdateRegion {
        int x, y, width, height;
        bool needsUpdate;
    };
    static UpdateRegion regions[9]; // 3x3 grid
    
public:
    /**
     * Initialize display
     */
    static bool initialize();
    
    /**
     * Set current screen
     * @param screen Screen to display
     */
    static void setScreen(Screen screen);
    
    /**
     * Set navigation mode
     * @param mode Navigation mode (normal, demo, vo2)
     */
    static void setMode(NavigationMode mode);
    
    /**
     * Update display with new data
     * @param data Current sensor and system data
     */
    static void update(const DisplayData& data);
    
    /**
     * Force full screen refresh
     */
    static void forceRefresh();
    
    /**
     * Navigate to next screen in current mode
     */
    static void nextScreen();
    
    /**
     * Navigate to previous screen in current mode
     */
    static void previousScreen();
    
    /**
     * Get current screen
     * @return Current screen
     */
    static Screen getCurrentScreen();
    
    /**
     * Get current mode
     * @return Current navigation mode
     */
    static NavigationMode getCurrentMode();
    
private:
    // Optimized drawing functions
    static void drawHeader(const DisplayData& data);
    static void drawMainMetrics(const DisplayData& data);
    static void drawRespiratoryData(const DisplayData& data);
    static void drawThresholds(const DisplayData& data);
    static void drawCalorimetry(const DisplayData& data);
    static void drawEnvironment(const DisplayData& data);
    static void drawParameters(const DisplayData& data);
    static void drawConnectivity(const DisplayData& data);
    static void drawISO(const DisplayData& data);
    static void drawDemoSelection(const DisplayData& data);
    static void drawSplashScreen();
    
    // Helper functions for optimized updates
    static void updateCell(int cellIndex, const String& value, const String& unit = "");
    static void updateHeader(bool forceUpdate = false);
    static void markRegionForUpdate(int cellIndex);
    static bool hasDataChanged(const DisplayData& newData, const DisplayData& oldData);
    static void drawGrid();
    static void drawBatteryIndicator(float voltage);
    static void drawConnectivityIcons(bool ble, bool wifi);
};