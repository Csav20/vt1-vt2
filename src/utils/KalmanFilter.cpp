#include "utils/KalmanFilter.h"
#include <math.h>

// Standard Kalman Filter Implementation
KalmanFilter::KalmanFilter(float initialEstimate, float processNoise, float measurementNoise)
    : x(initialEstimate), P(1.0f), Q(processNoise), R(measurementNoise), 
      K(0.0f), lastInnovation(0.0f), adaptiveMode(false) {
}

float KalmanFilter::update(float measurement) {
    // Prediction step
    float x_pred = x;  // No motion model, so predicted state equals current state
    float P_pred = P + Q;
    
    // Innovation
    lastInnovation = measurement - x_pred;
    
    // Innovation covariance
    float S = P_pred + R;
    
    // Kalman gain
    K = P_pred / S;
    
    // Update step
    x = x_pred + K * lastInnovation;
    P = (1.0f - K) * P_pred;
    
    // Adaptive noise adjustment if enabled
    if (adaptiveMode) {
        updateNoiseParameters(lastInnovation);
    }
    
    return x;
}

void KalmanFilter::reset(float initialEstimate) {
    x = initialEstimate;
    P = 1.0f;
    lastInnovation = 0.0f;
}

void KalmanFilter::setProcessNoise(float noise) {
    Q = noise;
}

void KalmanFilter::setMeasurementNoise(float noise) {
    R = noise;
}

void KalmanFilter::setAdaptiveMode(bool enable) {
    adaptiveMode = enable;
}

void KalmanFilter::updateNoiseParameters(float innovation) {
    // Adaptive noise based on innovation magnitude
    float innovationMagnitude = fabs(innovation);
    
    // Increase process noise if innovation is large (system changing rapidly)
    if (innovationMagnitude > 2.0f * sqrt(R)) {
        Q = min(Q * 1.1f, 1.0f);  // Cap at 1.0
    } else {
        Q = max(Q * 0.99f, 0.001f);  // Floor at 0.001
    }
    
    // Adjust measurement noise based on consistency
    static float innovationHistory[5] = {0};
    static int historyIndex = 0;
    
    innovationHistory[historyIndex] = innovationMagnitude;
    historyIndex = (historyIndex + 1) % 5;
    
    // Calculate variance of recent innovations
    float meanInnovation = 0;
    for (int i = 0; i < 5; i++) {
        meanInnovation += innovationHistory[i];
    }
    meanInnovation /= 5.0f;
    
    float innovationVariance = 0;
    for (int i = 0; i < 5; i++) {
        float diff = innovationHistory[i] - meanInnovation;
        innovationVariance += diff * diff;
    }
    innovationVariance /= 5.0f;
    
    // Adjust R based on innovation consistency
    if (innovationVariance > 0.5f) {
        R = min(R * 1.05f, 5.0f);  // Increase noise if inconsistent
    } else {
        R = max(R * 0.98f, 0.01f);  // Decrease noise if consistent
    }
}

// Quantum-Enhanced Kalman Filter Implementation
QuantumKalmanFilter::QuantumKalmanFilter(float initialEstimate, float baseNoise)
    : x(initialEstimate), P(1.0f), baseQ(baseNoise), R(0.1f), 
      quantumFactor(0.15f), uncertainty(baseNoise) {
}

float QuantumKalmanFilter::update(float measurement, float uncertaintyFactor) {
    // Calculate quantum-enhanced process noise
    float innovation = measurement - x;
    float quantumNoise = calculateQuantumNoise(innovation, uncertaintyFactor);
    float Q = baseQ + quantumNoise;
    
    // Prediction step with quantum uncertainty
    float x_pred = x;
    float P_pred = P + Q;
    
    // Innovation with quantum factor
    innovation = measurement - x_pred;
    
    // Kalman gain with quantum enhancement
    float K = P_pred / (P_pred + R * (1.0f + uncertaintyFactor));
    
    // Update with quantum correction term
    float quantumCorrection = quantumFactor * exp(-fabs(innovation) / uncertainty);
    x = x_pred + K * innovation + quantumCorrection * innovation;
    P = (1.0f - K) * P_pred;
    
    // Update uncertainty estimate
    uncertainty = 0.9f * uncertainty + 0.1f * fabs(innovation);
    
    return x;
}

void QuantumKalmanFilter::reset(float initialEstimate) {
    x = initialEstimate;
    P = 1.0f;
    uncertainty = baseQ;
}

void QuantumKalmanFilter::setQuantumFactor(float factor) {
    quantumFactor = constrain(factor, 0.0f, 1.0f);
}

float QuantumKalmanFilter::calculateQuantumNoise(float innovation, float uncertaintyFactor) {
    // Quantum uncertainty principle applied to metabolic measurements
    // Larger innovations suggest quantum-like uncertainty in biological systems
    float innovationMagnitude = fabs(innovation);
    
    // Base quantum noise proportional to uncertainty
    float baseQuantumNoise = quantumFactor * innovationMagnitude;
    
    // Scale by uncertainty factor (e.g., movement, stress, etc.)
    float scaledNoise = baseQuantumNoise * (1.0f + uncertaintyFactor);
    
    // Add harmonic oscillator component for biological rhythms
    static float phase = 0.0f;
    phase += 0.1f;  // Increment phase for oscillation
    float harmonicComponent = 0.1f * sin(phase) * quantumFactor;
    
    return scaledNoise + harmonicComponent;
}