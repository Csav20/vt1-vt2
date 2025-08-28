#pragma once

#include <Arduino.h>

/**
 * @brief Efficient moving average filter with configurable window size
 * 
 * Template-based implementation for different data types and window sizes
 * Optimized for real-time sensor data processing
 */
template<typename T, size_t WindowSize>
class MovingAverage {
public:
    MovingAverage() : index(0), count(0), sum(0) {
        for (size_t i = 0; i < WindowSize; i++) {
            buffer[i] = 0;
        }
    }
    
    T add(T value) {
        if (count < WindowSize) {
            count++;
        } else {
            sum -= buffer[index];
        }
        
        buffer[index] = value;
        sum += value;
        index = (index + 1) % WindowSize;
        
        return sum / count;
    }
    
    T getAverage() const {
        return count > 0 ? sum / count : 0;
    }
    
    void reset() {
        index = 0;
        count = 0;
        sum = 0;
        for (size_t i = 0; i < WindowSize; i++) {
            buffer[i] = 0;
        }
    }
    
    size_t getCount() const { return count; }
    bool isFull() const { return count == WindowSize; }
    
    T getVariance() const {
        if (count < 2) return 0;
        
        T mean = getAverage();
        T variance = 0;
        
        for (size_t i = 0; i < count; i++) {
            T diff = buffer[i] - mean;
            variance += diff * diff;
        }
        
        return variance / (count - 1);
    }
    
    T getStandardDeviation() const {
        return sqrt(getVariance());
    }
    
private:
    T buffer[WindowSize];
    size_t index;
    size_t count;
    T sum;
};

/**
 * @brief Adaptive moving average with automatic window size adjustment
 * 
 * Automatically adjusts window size based on signal characteristics
 * for optimal filtering performance
 */
template<typename T>
class AdaptiveMovingAverage {
public:
    AdaptiveMovingAverage(size_t minWindow = 3, size_t maxWindow = 20) 
        : minWindowSize(minWindow), maxWindowSize(maxWindow), currentWindowSize(minWindow) {
        buffer = new T[maxWindow];
        reset();
    }
    
    ~AdaptiveMovingAverage() {
        delete[] buffer;
    }
    
    T add(T value) {
        // Add new value
        if (count < maxWindowSize) {
            count++;
        } else {
            sum -= buffer[index];
        }
        
        buffer[index] = value;
        sum += value;
        index = (index + 1) % maxWindowSize;
        
        // Adapt window size based on signal stability
        adaptWindowSize();
        
        // Calculate average using current window size
        size_t effectiveCount = min(count, currentWindowSize);
        return sum / effectiveCount;
    }
    
    void reset() {
        index = 0;
        count = 0;
        sum = 0;
        currentWindowSize = minWindowSize;
        for (size_t i = 0; i < maxWindowSize; i++) {
            buffer[i] = 0;
        }
    }
    
    size_t getCurrentWindowSize() const { return currentWindowSize; }
    
private:
    T* buffer;
    size_t index;
    size_t count;
    T sum;
    size_t minWindowSize;
    size_t maxWindowSize;
    size_t currentWindowSize;
    
    void adaptWindowSize() {
        if (count < maxWindowSize) return;
        
        // Calculate signal stability (inverse of variance)
        T variance = calculateVariance();
        
        // Adjust window size based on stability
        if (variance < 0.1) {
            // Stable signal - use larger window
            currentWindowSize = min(currentWindowSize + 1, maxWindowSize);
        } else if (variance > 1.0) {
            // Unstable signal - use smaller window
            currentWindowSize = max(currentWindowSize - 1, minWindowSize);
        }
    }
    
    T calculateVariance() const {
        if (count < 2) return 0;
        
        T mean = sum / count;
        T variance = 0;
        
        for (size_t i = 0; i < count; i++) {
            T diff = buffer[i] - mean;
            variance += diff * diff;
        }
        
        return variance / (count - 1);
    }
};