#include "utils/GraphRenderer.h"
#include <Arduino.h>

GraphRenderer::GraphRenderer() 
    : tft(nullptr), dataBuffer(), timeBuffer(), minY(0), maxY(100), 
      autoScale(true), marginLeft(DEFAULT_MARGIN), marginRight(DEFAULT_MARGIN),
      marginTop(DEFAULT_MARGIN), marginBottom(DEFAULT_MARGIN),
      animationEnabled(false), animationSpeed(1), animationOffset(0),
      lastRenderTime(0), renderInterval(1000/10), needsRedraw(true),
      numHorizontalLines(0), numVerticalLines(0), numThresholdMarkers(0) {
}

GraphRenderer::GraphRenderer(TFT_eSPI* display, const GraphConfig& configParam) 
    : GraphRenderer() {
    initialize(display, configParam);
}

void GraphRenderer::initialize(TFT_eSPI* display, const GraphConfig& configParam) {
    tft = display;
    config = configParam;
    
    // Set default scale
    setManualScale(0, 100);
    
    Serial.println("Graph renderer initialized");
}

void GraphRenderer::setConfig(const GraphConfig& configParam) {
    config = configParam;
    needsRedraw = true;
}

void GraphRenderer::addDataPoint(float value) {
    uint32_t timestamp = millis();
    
    dataBuffer.push(value);
    timeBuffer.push(timestamp);
    
    if (autoScale) {
        updateScaling();
    }
    
    needsRedraw = true;
}

void GraphRenderer::render() {
    if (!tft || !needsUpdate()) return;
    
    uint32_t startTime = micros();
    
    renderBackground();
    if (config.showGrid) {
        renderGrid();
    }
    renderAxes();
    renderData();
    renderAnnotations();
    if (config.showLabels) {
        renderLabels();
    }
    
    lastRenderTime = millis();
    needsRedraw = false;
    
    uint32_t renderTime = micros() - startTime;
    if (renderTime > 50000) { // Warn if rendering takes more than 50ms
        Serial.printf("WARNING: Graph render took %lu us\n", renderTime);
    }
}

void GraphRenderer::renderBackground() {
    if (!tft) return;
    
    tft->fillRect(config.x, config.y, config.width, config.height, config.backgroundColor);
    
    // Draw border
    tft->drawRect(config.x, config.y, config.width, config.height, config.gridColor);
}

void GraphRenderer::renderGrid() {
    if (!tft || !config.showGrid) return;
    
    uint16_t plotX = config.x + marginLeft;
    uint16_t plotY = config.y + marginTop;
    uint16_t plotWidth = config.width - marginLeft - marginRight;
    uint16_t plotHeight = config.height - marginTop - marginBottom;
    
    // Vertical grid lines (time axis)
    int numVLines = 5;
    for (int i = 1; i < numVLines; i++) {
        uint16_t x = plotX + (plotWidth * i) / numVLines;
        tft->drawLine(x, plotY, x, plotY + plotHeight, config.gridColor);
    }
    
    // Horizontal grid lines (value axis)
    int numHLines = 4;
    for (int i = 1; i < numHLines; i++) {
        uint16_t y = plotY + (plotHeight * i) / numHLines;
        tft->drawLine(plotX, y, plotX + plotWidth, y, config.gridColor);
    }
}

void GraphRenderer::renderAxes() {
    if (!tft) return;
    
    uint16_t plotX = config.x + marginLeft;
    uint16_t plotY = config.y + marginTop;
    uint16_t plotWidth = config.width - marginLeft - marginRight;
    uint16_t plotHeight = config.height - marginTop - marginBottom;
    
    // Draw axes
    tft->drawLine(plotX, plotY + plotHeight, plotX + plotWidth, plotY + plotHeight, config.lineColor); // X-axis
    tft->drawLine(plotX, plotY, plotX, plotY + plotHeight, config.lineColor); // Y-axis
}

void GraphRenderer::renderData() {
    if (!tft || dataBuffer.size() < 2) return;
    
    uint16_t plotX = config.x + marginLeft;
    uint16_t plotY = config.y + marginTop;
    uint16_t plotWidth = config.width - marginLeft - marginRight;
    uint16_t plotHeight = config.height - marginTop - marginBottom;
    
    // Draw data points
    for (size_t i = 1; i < dataBuffer.size(); i++) {
        uint16_t x1 = plotX + (plotWidth * (i - 1)) / (dataBuffer.size() - 1);
        uint16_t y1 = valueToPixelY(dataBuffer[i - 1]);
        uint16_t x2 = plotX + (plotWidth * i) / (dataBuffer.size() - 1);
        uint16_t y2 = valueToPixelY(dataBuffer[i]);
        
        if (isPointVisible(x1, y1) && isPointVisible(x2, y2)) {
            drawOptimizedLine(x1, y1, x2, y2, config.lineColor);
        }
    }
}

void GraphRenderer::renderAnnotations() {
    if (!tft) return;
    
    // Draw horizontal lines
    for (int i = 0; i < numHorizontalLines; i++) {
        uint16_t y = valueToPixelY(horizontalLines[i].value);
        uint16_t plotX = config.x + marginLeft;
        uint16_t plotWidth = config.width - marginLeft - marginRight;
        
        tft->drawLine(plotX, y, plotX + plotWidth, y, horizontalLines[i].color);
        
        if (strlen(horizontalLines[i].label) > 0) {
            tft->setTextColor(horizontalLines[i].color);
            tft->setTextSize(1);
            tft->drawString(horizontalLines[i].label, plotX + 5, y - 10);
        }
    }
}

void GraphRenderer::renderLabels() {
    if (!tft) return;
    
    // Draw axis labels
    tft->setTextColor(config.textColor);
    tft->setTextSize(1);
    
    // Y-axis labels
    char labelStr[16];
    snprintf(labelStr, sizeof(labelStr), "%.1f", maxY);
    tft->drawString(labelStr, config.x + 2, config.y + marginTop);
    
    snprintf(labelStr, sizeof(labelStr), "%.1f", minY);
    tft->drawString(labelStr, config.x + 2, config.y + config.height - marginBottom - 10);
}

uint16_t GraphRenderer::valueToPixelY(float value) const {
    uint16_t plotY = config.y + marginTop;
    uint16_t plotHeight = config.height - marginTop - marginBottom;
    
    float normalizedValue = (value - minY) / (maxY - minY);
    normalizedValue = constrain(normalizedValue, 0.0f, 1.0f);
    
    return plotY + plotHeight - (uint16_t)(normalizedValue * plotHeight);
}

bool GraphRenderer::isPointVisible(uint16_t x, uint16_t y) const {
    return (x >= config.x && x < config.x + config.width &&
            y >= config.y && y < config.y + config.height);
}

void GraphRenderer::drawOptimizedLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    if (!tft) return;
    
    // Simple line drawing - could be optimized further
    tft->drawLine(x1, y1, x2, y2, color);
}

void GraphRenderer::setDataRange(float minValue, float maxValue) {
    setManualScale(minValue, maxValue);
}

void GraphRenderer::setManualScale(float minVal, float maxVal) {
    minY = minVal;
    maxY = maxVal;
    autoScale = false;
    needsRedraw = true;
}

void GraphRenderer::enableAutoScale(bool enable) {
    autoScale = enable;
    if (enable) {
        updateScaling();
    }
}

void GraphRenderer::updateScaling() {
    if (dataBuffer.size() == 0) return;
    
    float newMin = dataBuffer.minimum();
    float newMax = dataBuffer.maximum();
    
    // Add some padding
    float range = newMax - newMin;
    if (range < 1.0f) range = 1.0f; // Minimum range
    
    minY = newMin - range * 0.1f;
    maxY = newMax + range * 0.1f;
    
    needsRedraw = true;
}

void GraphRenderer::addHorizontalLine(float value, uint16_t color, const char* label) {
    if (numHorizontalLines < MAX_ANNOTATIONS) {
        horizontalLines[numHorizontalLines].value = value;
        horizontalLines[numHorizontalLines].color = color;
        
        if (label) {
            strncpy(horizontalLines[numHorizontalLines].label, label, 15);
            horizontalLines[numHorizontalLines].label[15] = '\0';
        } else {
            horizontalLines[numHorizontalLines].label[0] = '\0';
        }
        
        numHorizontalLines++;
        needsRedraw = true;
    }
}

void GraphRenderer::clearData() {
    dataBuffer.clear();
    timeBuffer.clear();
    needsRedraw = true;
}

void GraphRenderer::clearAnnotations() {
    numHorizontalLines = 0;
    numVerticalLines = 0;
    numThresholdMarkers = 0;
    needsRedraw = true;
}

bool GraphRenderer::needsUpdate() const {
    return needsRedraw || (millis() - lastRenderTime > renderInterval);
}

void GraphRenderer::setUpdateFrequency(uint8_t hz) {
    renderInterval = hz > 0 ? 1000 / hz : 100; // Default to 10Hz if invalid
}

// Stub implementations for remaining methods
void GraphRenderer::setTimeWindow(uint32_t windowSeconds) {
    // Implementation would manage time-based data retention
}

void GraphRenderer::renderPartial(uint16_t startX, uint16_t endX) {
    // Implementation would render only a portion of the graph
    render(); // For now, just render the whole graph
}

void GraphRenderer::addVerticalLine(uint32_t timeOffset, uint16_t color, const char* label) {
    // Implementation would add vertical time markers
}

void GraphRenderer::addThresholdMarker(float value, uint16_t color, bool above) {
    // Implementation would add threshold indicators
}

void GraphRenderer::enableAnimation(bool enable) {
    animationEnabled = enable;
}

void GraphRenderer::setAnimationSpeed(uint8_t speed) {
    animationSpeed = constrain(speed, 1, 10);
}