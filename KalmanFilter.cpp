#include "KalmanFilter.h"

UnifiedKalmanFilter::UnifiedKalmanFilter(float processNoise, float measurementNoise, float initialEstimate) 
    : processNoise(processNoise)
    , measurementNoise(measurementNoise)
    , estimate(initialEstimate)
    , errorCovariance(1.0)
    , initialized(false) {
}

float UnifiedKalmanFilter::update(float measurement) {
    if (!initialized) {
        estimate = measurement;
        initialized = true;
        return estimate;
    }
    
    // Prediction step
    // estimate remains the same (no process model)
    errorCovariance += processNoise;
    
    // Update step
    float kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
    estimate = estimate + kalmanGain * (measurement - estimate);
    errorCovariance = (1.0 - kalmanGain) * errorCovariance;
    
    return estimate;
}

float UnifiedKalmanFilter::getEstimate() const {
    return estimate;
}

void UnifiedKalmanFilter::reset(float newEstimate) {
    estimate = newEstimate;
    errorCovariance = 1.0;
    initialized = false;
}

void UnifiedKalmanFilter::setNoiseParameters(float newProcessNoise, float newMeasurementNoise) {
    processNoise = newProcessNoise;
    measurementNoise = newMeasurementNoise;
}

bool UnifiedKalmanFilter::isInitialized() const {
    return initialized;
}