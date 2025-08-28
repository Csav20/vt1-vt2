#pragma once

#include <Arduino.h>

/**
 * @brief High-performance circular buffer for time-series data
 * 
 * Optimized for real-time data collection and analysis
 * Template-based for different data types
 */
template<typename T, size_t Size>
class CircularBuffer {
public:
    CircularBuffer() : head(0), count(0) {}
    
    void push(const T& item) {
        buffer[head] = item;
        head = (head + 1) % Size;
        
        if (count < Size) {
            count++;
        }
    }
    
    T& operator[](size_t index) {
        if (index >= count) {
            // Return reference to first element if index out of bounds
            return buffer[0];
        }
        
        // Calculate actual index in circular buffer
        size_t actualIndex = (head + Size - count + index) % Size;
        return buffer[actualIndex];
    }
    
    const T& operator[](size_t index) const {
        if (index >= count) {
            return buffer[0];
        }
        
        size_t actualIndex = (head + Size - count + index) % Size;
        return buffer[actualIndex];
    }
    
    size_t size() const { return count; }
    size_t capacity() const { return Size; }
    bool empty() const { return count == 0; }
    bool full() const { return count == Size; }
    
    void clear() {
        head = 0;
        count = 0;
    }
    
    T getLatest() const {
        if (count == 0) return T();
        return buffer[(head + Size - 1) % Size];
    }
    
    T getOldest() const {
        if (count == 0) return T();
        return buffer[(head + Size - count) % Size];
    }
    
    // Statistical functions
    T average() const {
        if (count == 0) return T();
        
        T sum = T();
        for (size_t i = 0; i < count; i++) {
            sum += (*this)[i];
        }
        return sum / count;
    }
    
    T minimum() const {
        if (count == 0) return T();
        
        T min_val = (*this)[0];
        for (size_t i = 1; i < count; i++) {
            if ((*this)[i] < min_val) {
                min_val = (*this)[i];
            }
        }
        return min_val;
    }
    
    T maximum() const {
        if (count == 0) return T();
        
        T max_val = (*this)[0];
        for (size_t i = 1; i < count; i++) {
            if ((*this)[i] > max_val) {
                max_val = (*this)[i];
            }
        }
        return max_val;
    }
    
    T variance() const {
        if (count < 2) return T();
        
        T mean = average();
        T var = T();
        
        for (size_t i = 0; i < count; i++) {
            T diff = (*this)[i] - mean;
            var += diff * diff;
        }
        
        return var / (count - 1);
    }
    
    T standardDeviation() const {
        return sqrt(variance());
    }
    
    // Find peaks and valleys
    bool isPeak(size_t index, size_t window = 1) const {
        if (index < window || index >= count - window) return false;
        
        T value = (*this)[index];
        for (size_t i = index - window; i <= index + window; i++) {
            if (i != index && (*this)[i] >= value) return false;
        }
        return true;
    }
    
    bool isValley(size_t index, size_t window = 1) const {
        if (index < window || index >= count - window) return false;
        
        T value = (*this)[index];
        for (size_t i = index - window; i <= index + window; i++) {
            if (i != index && (*this)[i] <= value) return false;
        }
        return true;
    }
    
    // Calculate slope between two points
    float slope(size_t startIndex, size_t endIndex) const {
        if (startIndex >= count || endIndex >= count || startIndex == endIndex) {
            return 0.0f;
        }
        
        float deltaY = static_cast<float>((*this)[endIndex] - (*this)[startIndex]);
        float deltaX = static_cast<float>(endIndex - startIndex);
        
        return deltaY / deltaX;
    }
    
    // Find linear trend over entire buffer
    float trend() const {
        if (count < 2) return 0.0f;
        return slope(0, count - 1);
    }
    
    // Copy data to array (useful for external processing)
    void copyToArray(T* array, size_t maxSize) const {
        size_t copyCount = min(count, maxSize);
        for (size_t i = 0; i < copyCount; i++) {
            array[i] = (*this)[i];
        }
    }
    
private:
    T buffer[Size];
    size_t head;
    size_t count;
};