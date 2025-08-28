#pragma once

#include <Arduino.h>
#include "SensorManager.h"
#include "utils/CircularBuffer.h"

/**
 * @brief Advanced metabolic calculations with optimized algorithms
 * 
 * Features:
 * - Optimized VO2/VCO2 calculations with pre-computed constants
 * - Ventilatory threshold detection (VT1, VT2)
 * - Multiple calculation methods (Wasserman, Beaver, V-slope)
 * - Real-time metabolic efficiency tracking
 */
class MetabolicCalculator {
public:
    struct MetabolicData {
        // Primary metabolic measurements
        float vo2;                  // ml/kg/min
        float vo2Max;               // ml/kg/min
        float vco2;                 // ml/kg/min
        float rer;                  // Respiratory Exchange Ratio
        float energyExpenditure;    // kcal/min
        
        // Substrate utilization
        float carbPercentage;       // % carbohydrate oxidation
        float fatPercentage;        // % fat oxidation
        float proteinPercentage;    // % protein oxidation
        
        // Ventilatory parameters
        float ventilationRate;      // L/min
        float respiratoryRate;      // breaths/min
        float tidalVolume;          // ml
        
        // Thresholds
        float vt1;                  // First ventilatory threshold
        float vt2;                  // Second ventilatory threshold
        bool vt1Detected;           // VT1 detection status
        bool vt2Detected;           // VT2 detection status
        
        // Efficiency metrics
        float vo2Efficiency;        // ml/kg/min per W
        float economyOfMovement;    // ml/kg per meter
        float metabolicCost;        // J/kg per meter
        
        // Quality indicators
        float dataQuality;          // Overall data quality 0-1
        uint32_t timestamp;         // Calculation timestamp
    };

    struct ThresholdData {
        float intensity;            // % of VO2max
        float heartRate;            // bpm
        float lactate;              // mmol/L (estimated)
        float rer;                  // RER at threshold
        bool isValid;               // Threshold validity
    };

    // Core calculation functions
    static void configure(float weightKg, float heightCm, int ageYears, bool hasCO2Sensor = false);
    static bool calculate(const SensorManager::SensorData& input, MetabolicData& output);
    
    // Threshold detection methods
    static bool detectVT1(ThresholdData& vt1Data);
    static bool detectVT2(ThresholdData& vt2Data);
    static void resetThresholdDetection();
    
    // Multiple calculation methods for validation
    static float calculateVO2Wasserman(float ventilation, float o2Percent);
    static float calculateVO2Beaver(float ventilation, float o2Percent, float co2Percent);
    static float calculateVO2VSlope(float vco2, float vo2);
    
    // Optimized calculations with pre-computed constants
    static float calculateVentilationBTPS(float flowRate, float temperature, float pressure);
    static float calculateMetabolicEquivalent(float vo2, float weight);
    static float calculateCaloriesFromRER(float vo2, float vco2, float weight);
    
    // Advanced analysis
    static void updateEfficiencyMetrics(float workload, float speed);
    static void analyzeBreathingPattern(const SensorManager::SensorData& input);
    static float estimateLactateFromVO2(float vo2, float vo2Max);
    
    // Data validation and quality assessment
    static bool validateMetabolicData(const MetabolicData& data);
    static float assessDataQuality(const SensorManager::SensorData& input);
    
    // Utility functions
    static void getRecommendedZones(float vo2Max, float zones[5]);
    static const char* getIntensityZone(float vo2, float vo2Max);
    static void exportData(const MetabolicData& data, char* jsonBuffer, size_t bufferSize);

private:
    // Subject configuration
    static float weight;            // kg
    static float height;            // cm
    static int age;                 // years
    static bool hasCO2Sensor;       // CO2 sensor availability
    
    // Pre-calculated constants for optimization
    static const float STPD_FACTOR;         // Standard temp/pressure correction
    static const float BTPS_FACTOR;         // Body temp/pressure correction
    static const float O2_DENSITY;          // kg/m³
    static const float CO2_DENSITY;         // kg/m³
    static const float CALORIC_EQUIV_O2;    // kcal/L O2
    static const float CALORIC_EQUIV_CO2;   // kcal/L CO2
    
    // Data buffers for threshold detection
    static CircularBuffer<float, 60> vo2Buffer;         // 3 minutes at 20Hz
    static CircularBuffer<float, 60> vco2Buffer;
    static CircularBuffer<float, 60> ventilationBuffer;
    static CircularBuffer<float, 60> rerBuffer;
    
    // Threshold detection state
    static bool thresholdDetectionActive;
    static float baselineVO2;
    static float peakVO2;
    static unsigned long detectionStartTime;
    
    // Filter parameters
    static float kalmanQ;           // Process noise
    static float kalmanR;           // Measurement noise
    
    // Helper functions
    static void updateBuffers(float vo2, float vco2, float ventilation);
    static float calculateSlope(const CircularBuffer<float, 60>& xData, 
                               const CircularBuffer<float, 60>& yData, 
                               int startIndex, int endIndex);
    static bool detectBreakpoint(const CircularBuffer<float, 60>& data, 
                                float& breakpointValue, int& breakpointIndex);
    static float smoothData(const CircularBuffer<float, 60>& data, int index, int window);
    static void applyEnvironmentalCorrection(float& vo2, float& vco2, 
                                           float temperature, float pressure, float humidity);
    
    // Validation ranges
    static const float MIN_VO2;
    static const float MAX_VO2;
    static const float MIN_RER;
    static const float MAX_RER;
    static const float MIN_VENTILATION;
    static const float MAX_VENTILATION;
};