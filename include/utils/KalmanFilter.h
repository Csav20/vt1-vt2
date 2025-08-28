#pragma once

#include <Arduino.h>

/**
 * @brief Optimized Kalman filter implementation for sensor data
 * 
 * Features:
 * - Adaptive noise parameters
 * - Quantum-enhanced filtering for metabolic data
 * - Real-time parameter tuning
 */
class KalmanFilter {
public:
    KalmanFilter(float initialEstimate = 0.0f, float processNoise = 0.01f, float measurementNoise = 0.1f);
    
    float update(float measurement);
    void reset(float initialEstimate = 0.0f);
    void setProcessNoise(float noise);
    void setMeasurementNoise(float noise);
    void setAdaptiveMode(bool enable);
    
    float getEstimate() const { return x; }
    float getErrorCovariance() const { return P; }
    float getInnovation() const { return lastInnovation; }
    
private:
    float x;            // State estimate
    float P;            // Error covariance
    float Q;            // Process noise
    float R;            // Measurement noise
    float K;            // Kalman gain
    float lastInnovation;
    bool adaptiveMode;
    
    void updateNoiseParameters(float innovation);
};

/**
 * @brief Quantum-enhanced Kalman filter for metabolic measurements
 * 
 * Incorporates quantum uncertainty principles for better estimation
 * of rapidly changing metabolic parameters like lactate and hormones
 */
class QuantumKalmanFilter {
public:
    QuantumKalmanFilter(float initialEstimate = 0.0f, float baseNoise = 0.01f);
    
    float update(float measurement, float uncertaintyFactor = 1.0f);
    void reset(float initialEstimate = 0.0f);
    void setQuantumFactor(float factor);
    
    float getEstimate() const { return x; }
    float getUncertainty() const { return uncertainty; }
    
private:
    float x;            // State estimate
    float P;            // Error covariance
    float baseQ;        // Base process noise
    float R;            // Measurement noise
    float quantumFactor; // Quantum enhancement factor
    float uncertainty;   // Current uncertainty level
    
    float calculateQuantumNoise(float innovation, float uncertaintyFactor);
};