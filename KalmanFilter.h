#pragma once

/**
 * Unified Kalman Filter Implementation
 * Consolidates FiltroKalman, FiltroKalmanGauss, FiltroKalmanCuantico, and KalmanFilter
 * Standardized to English naming
 */
class UnifiedKalmanFilter {
private:
    float processNoise;         // Process noise (Q)
    float measurementNoise;     // Measurement noise (R) 
    float estimate;             // Current estimate
    float errorCovariance;      // Error covariance (P)
    bool initialized;           // Initialization flag
    
public:
    /**
     * Constructor
     * @param processNoise Process noise value (default: 0.01)
     * @param measurementNoise Measurement noise value (default: 0.1)
     * @param initialEstimate Initial state estimate (default: 0.0)
     */
    UnifiedKalmanFilter(float processNoise = 0.01, float measurementNoise = 0.1, float initialEstimate = 0.0);
    
    /**
     * Update filter with new measurement
     * @param measurement New measurement value
     * @return Filtered estimate
     */
    float update(float measurement);
    
    /**
     * Get current estimate
     * @return Current filtered value
     */
    float getEstimate() const;
    
    /**
     * Reset filter to initial state
     * @param newEstimate New initial estimate (optional)
     */
    void reset(float newEstimate = 0.0);
    
    /**
     * Set noise parameters
     * @param newProcessNoise Process noise value
     * @param newMeasurementNoise Measurement noise value
     */
    void setNoiseParameters(float newProcessNoise, float newMeasurementNoise);
    
    /**
     * Check if filter is initialized
     * @return true if initialized
     */
    bool isInitialized() const;
};