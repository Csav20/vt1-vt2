#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "CircularBuffer.h"

/**
 * @brief High-performance graph renderer for real-time data visualization
 * 
 * Features:
 * - Optimized rendering with partial updates
 * - Multiple graph types (line, area, bar)
 * - Automatic scaling and grid generation
 * - Smooth animations and transitions
 */
class GraphRenderer {
public:
    enum GraphType {
        LINE_GRAPH,
        AREA_GRAPH,
        BAR_GRAPH,
        SCATTER_PLOT
    };

    struct GraphConfig {
        uint16_t x, y;              // Position
        uint16_t width, height;     // Dimensions
        uint16_t lineColor;         // Primary line color
        uint16_t fillColor;         // Fill color for area graphs
        uint16_t backgroundColor;   // Background color
        uint16_t gridColor;         // Grid color
        uint16_t textColor;         // Text color
        uint8_t lineWidth;          // Line thickness
        bool showGrid;              // Show grid lines
        bool showLabels;            // Show axis labels
        bool showValues;            // Show data point values
        bool enableSmoothing;       // Enable line smoothing
        GraphType type;             // Graph type
    };

    GraphRenderer();
    GraphRenderer(TFT_eSPI* display, const GraphConfig& config);
    
    void initialize(TFT_eSPI* display, const GraphConfig& config);
    void setConfig(const GraphConfig& config);
    
    // Data management
    void addDataPoint(float value);
    void setDataRange(float minValue, float maxValue);
    void setTimeWindow(uint32_t windowSeconds);
    void clearData();
    
    // Rendering
    void render();
    void renderPartial(uint16_t startX, uint16_t endX);
    void renderToBuffer(uint16_t* buffer);
    
    // Auto-scaling
    void enableAutoScale(bool enable);
    void setManualScale(float minY, float maxY);
    void setMargins(uint8_t left, uint8_t right, uint8_t top, uint8_t bottom);
    
    // Annotations
    void addHorizontalLine(float value, uint16_t color, const char* label = nullptr);
    void addVerticalLine(uint32_t timeOffset, uint16_t color, const char* label = nullptr);
    void addThresholdMarker(float value, uint16_t color, bool above = true);
    void clearAnnotations();
    
    // Animation
    void enableAnimation(bool enable);
    void setAnimationSpeed(uint8_t speed);
    
    // Performance optimization
    void setUpdateFrequency(uint8_t hz);
    bool needsUpdate() const;
    uint32_t getLastRenderTime() const;

private:
    TFT_eSPI* tft;
    GraphConfig config;
    CircularBuffer<float, 240> dataBuffer;  // Store up to 240 data points
    CircularBuffer<uint32_t, 240> timeBuffer; // Corresponding timestamps
    
    // Scaling parameters
    float minY, maxY;
    bool autoScale;
    uint8_t marginLeft, marginRight, marginTop, marginBottom;
    
    // Animation state
    bool animationEnabled;
    uint8_t animationSpeed;
    float animationOffset;
    
    // Performance tracking
    uint32_t lastRenderTime;
    uint32_t renderInterval;
    bool needsRedraw;
    
    // Annotations
    struct HorizontalLine {
        float value;
        uint16_t color;
        char label[16];
    };
    
    struct VerticalLine {
        uint32_t timeOffset;
        uint16_t color;
        char label[16];
    };
    
    struct ThresholdMarker {
        float value;
        uint16_t color;
        bool above;
    };
    
    HorizontalLine horizontalLines[4];
    VerticalLine verticalLines[4];
    ThresholdMarker thresholdMarkers[4];
    uint8_t numHorizontalLines;
    uint8_t numVerticalLines;
    uint8_t numThresholdMarkers;
    
    // Internal rendering functions
    void renderBackground();
    void renderGrid();
    void renderAxes();
    void renderData();
    void renderAnnotations();
    void renderLabels();
    
    // Data processing
    void updateScaling();
    void smoothData();
    uint16_t valueToPixelY(float value) const;
    uint16_t timeToPixelX(uint32_t timestamp) const;
    float pixelToValueY(uint16_t pixel) const;
    uint32_t pixelToTimeX(uint16_t pixel) const;
    
    // Optimization helpers
    bool isPointVisible(uint16_t x, uint16_t y) const;
    void clipLine(int16_t& x1, int16_t& y1, int16_t& x2, int16_t& y2) const;
    void drawOptimizedLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
    
    // Constants
    static const uint8_t MAX_ANNOTATIONS = 4;
    static const uint8_t DEFAULT_MARGIN = 10;
    static const uint32_t DEFAULT_TIME_WINDOW = 60000; // 60 seconds
};