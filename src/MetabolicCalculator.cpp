#include "MetabolicCalculator.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// Static member definitions
float MetabolicCalculator::weight = 75.0f;
float MetabolicCalculator::height = 175.0f;
int MetabolicCalculator::age = 25;
bool MetabolicCalculator::hasCO2Sensor = false;
CircularBuffer<float, 60> MetabolicCalculator::vo2Buffer;
CircularBuffer<float, 60> MetabolicCalculator::vco2Buffer;
CircularBuffer<float, 60> MetabolicCalculator::ventilationBuffer;
CircularBuffer<float, 60> MetabolicCalculator::rerBuffer;
bool MetabolicCalculator::thresholdDetectionActive = false;
float MetabolicCalculator::baselineVO2 = 0.0f;
float MetabolicCalculator::peakVO2 = 0.0f;
unsigned long MetabolicCalculator::detectionStartTime = 0;
float MetabolicCalculator::kalmanQ = KALMAN_Q_DEFAULT;
float MetabolicCalculator::kalmanR = KALMAN_R_DEFAULT;

// Pre-calculated constants
const float MetabolicCalculator::STPD_FACTOR = STPD_FACTOR;
const float MetabolicCalculator::BTPS_FACTOR = BTPS_FACTOR;
const float MetabolicCalculator::O2_DENSITY = O2_DENSITY_STP;
const float MetabolicCalculator::CO2_DENSITY = CO2_DENSITY_STP;
const float MetabolicCalculator::CALORIC_EQUIV_O2 = CALORIC_EQUIV_O2;
const float MetabolicCalculator::CALORIC_EQUIV_CO2 = CALORIC_EQUIV_CO2;

// Validation ranges
const float MetabolicCalculator::MIN_VO2 = VO2_MIN_VALUE;
const float MetabolicCalculator::MAX_VO2 = VO2_MAX_VALUE;
const float MetabolicCalculator::MIN_RER = RER_MIN_VALUE;
const float MetabolicCalculator::MAX_RER = RER_MAX_VALUE;
const float MetabolicCalculator::MIN_VENTILATION = 5.0f;
const float MetabolicCalculator::MAX_VENTILATION = 200.0f;

void MetabolicCalculator::configure(float weightKg, float heightCm, int ageYears, bool co2Available) {
    weight = weightKg;
    height = heightCm;
    age = ageYears;
    hasCO2Sensor = co2Available;
    
    Serial.printf("Metabolic calculator configured: %.1fkg, %.1fcm, %d years, CO2: %s\n",
                 weight, height, age, hasCO2Sensor ? "Yes" : "No");
    
    // Reset buffers
    vo2Buffer.clear();
    vco2Buffer.clear();
    ventilationBuffer.clear();
    rerBuffer.clear();
    
    thresholdDetectionActive = false;
}

bool MetabolicCalculator::calculate(const SensorManager::SensorData& input, MetabolicData& output) {
    // Clear output structure
    memset(&output, 0, sizeof(MetabolicData));
    output.timestamp = millis();
    
    // Basic validation of input data
    if (output.timestamp == 0 || !input.sensorValid[0]) {
        output.dataQuality = 0.0f;
        return false;
    }
    
    // Calculate ventilation rate from pressure sensor (simplified)
    float flowRate = SensorManager::calculateFlowRate(input.pressure, input.ambientTemp);
    output.ventilationRate = calculateVentilationBTPS(flowRate, input.ambientTemp, input.ambientPressure);
    
    // Estimate tidal volume and respiratory rate (simplified)
    output.tidalVolume = output.ventilationRate / 15.0f * 1000.0f; // Assume 15 breaths/min
    output.respiratoryRate = 15.0f; // Placeholder
    
    // Calculate VO2 using multiple methods for validation
    float vo2_wasserman = calculateVO2Wasserman(output.ventilationRate, input.o2Percent);
    float vo2_beaver = vo2_wasserman; // Placeholder - would use CO2 if available
    
    // Use primary method (Wasserman for now)
    output.vo2 = vo2_wasserman;
    
    // Calculate VCO2 if CO2 sensor is available
    if (hasCO2Sensor && input.co2Ppm > 0) {
        output.vco2 = calculateVCO2FromConcentration(output.ventilationRate, input.co2Ppm);
        output.rer = output.vco2 / output.vo2;
    } else {
        // Estimate VCO2 and RER based on exercise intensity
        float intensityFactor = output.vo2 / 40.0f; // Assume VO2max around 40
        output.rer = 0.7f + (0.3f * intensityFactor); // RER from 0.7 to 1.0
        output.vco2 = output.vo2 * output.rer;
    }
    
    // Clamp values to valid ranges
    output.vo2 = constrain(output.vo2, MIN_VO2, MAX_VO2);
    output.vco2 = constrain(output.vco2, MIN_VO2 * MIN_RER, MAX_VO2 * MAX_RER);
    output.rer = constrain(output.rer, MIN_RER, MAX_RER);
    
    // Calculate energy expenditure using Weir equation
    output.energyExpenditure = calculateCaloriesFromRER(output.vo2, output.vco2, weight);
    
    // Calculate substrate utilization based on RER
    calculateSubstrateUtilization(output.rer, output.carbPercentage, 
                                 output.fatPercentage, output.proteinPercentage);
    
    // Update VO2 max estimate
    static float vo2MaxEstimate = 40.0f;
    if (output.vo2 > vo2MaxEstimate) {
        vo2MaxEstimate = output.vo2 * 1.1f; // Slightly above current max
    }
    output.vo2Max = vo2MaxEstimate;
    
    // Update data buffers for threshold detection
    updateBuffers(output.vo2, output.vco2, output.ventilationRate);
    
    // Detect thresholds if enough data is available
    ThresholdData vt1Data, vt2Data;
    output.vt1Detected = detectVT1(vt1Data);
    output.vt2Detected = detectVT2(vt2Data);
    
    if (output.vt1Detected) {
        output.vt1 = vt1Data.intensity * output.vo2Max / 100.0f;
    }
    if (output.vt2Detected) {
        output.vt2 = vt2Data.intensity * output.vo2Max / 100.0f;
    }
    
    // Calculate efficiency metrics
    output.vo2Efficiency = output.vo2 / weight; // ml/kg/min
    output.economyOfMovement = output.vo2; // Placeholder - would need speed data
    output.metabolicCost = output.energyExpenditure * 4.184f * 1000.0f; // Convert kcal to J
    
    // Assess data quality
    output.dataQuality = assessDataQuality(input);
    
    return validateMetabolicData(output);
}

float MetabolicCalculator::calculateVO2Wasserman(float ventilation, float o2Percent) {
    // Wasserman method: VO2 = VE * (FIO2 - FEO2) / 100
    // Where FIO2 = 20.93% (atmospheric), FEO2 = measured O2%
    
    float fio2 = 20.93f; // Atmospheric O2 percentage
    float feo2 = o2Percent;
    float o2Diff = fio2 - feo2;
    
    // Apply STPD correction and convert to ml/kg/min
    float vo2_L_min = (ventilation * o2Diff / 100.0f) * STPD_FACTOR;
    float vo2_ml_kg_min = (vo2_L_min * 1000.0f) / weight;
    
    return vo2_ml_kg_min;
}

float MetabolicCalculator::calculateVentilationBTPS(float flowRate, float temperature, float pressure) {
    // Convert flow rate to BTPS (Body Temperature Pressure Saturated) conditions
    // BTPS factors account for body temperature (37°C) and water vapor pressure
    
    float tempKelvin = temperature + 273.15f;
    float bodyTempKelvin = 37.0f + 273.15f;
    float pressureKPa = pressure / 1000.0f;
    
    // Temperature correction factor
    float tempFactor = bodyTempKelvin / tempKelvin;
    
    // Pressure correction factor (subtract water vapor pressure at body temp)
    float waterVaporPressure = 6.27f; // kPa at 37°C
    float pressureFactor = (pressureKPa - waterVaporPressure) / (101.325f - waterVaporPressure);
    
    return flowRate * tempFactor * pressureFactor;
}

float MetabolicCalculator::calculateCaloriesFromRER(float vo2, float vco2, float weightKg) {
    // Weir equation: Energy (kcal/min) = 3.941*VO2 + 1.106*VCO2
    // Where VO2 and VCO2 are in L/min
    
    float vo2_L_min = (vo2 * weightKg) / 1000.0f;
    float vco2_L_min = (vco2 * weightKg) / 1000.0f;
    
    return (3.941f * vo2_L_min) + (1.106f * vco2_L_min);
}

void MetabolicCalculator::calculateSubstrateUtilization(float rer, float& carbPercent, 
                                                      float& fatPercent, float& proteinPercent) {
    // Simplified substrate utilization based on RER
    // Assumes negligible protein oxidation for exercise
    
    proteinPercent = 0.0f; // Assume minimal protein oxidation during exercise
    
    if (rer <= 0.7f) {
        // Pure fat oxidation
        fatPercent = 100.0f;
        carbPercent = 0.0f;
    } else if (rer >= 1.0f) {
        // Pure carbohydrate oxidation
        fatPercent = 0.0f;
        carbPercent = 100.0f;
    } else {
        // Mixed substrate oxidation
        float factor = (rer - 0.7f) / 0.3f;
        carbPercent = factor * 100.0f;
        fatPercent = (1.0f - factor) * 100.0f;
    }
}

float MetabolicCalculator::calculateVCO2FromConcentration(float ventilation, float co2Ppm) {
    // Calculate VCO2 from expired CO2 concentration
    // VCO2 = VE * (FECO2 - FICO2) / 100
    
    float fico2 = 0.04f; // Atmospheric CO2 percentage (~400 ppm)
    float feco2 = co2Ppm / 10000.0f; // Convert ppm to percentage
    float co2Diff = feco2 - fico2;
    
    // Apply STPD correction and convert to ml/kg/min
    float vco2_L_min = (ventilation * co2Diff / 100.0f) * STPD_FACTOR;
    float vco2_ml_kg_min = (vco2_L_min * 1000.0f) / weight;
    
    return vco2_ml_kg_min;
}

void MetabolicCalculator::updateBuffers(float vo2, float vco2, float ventilation) {
    vo2Buffer.push(vo2);
    vco2Buffer.push(vco2);
    ventilationBuffer.push(ventilation);
    
    if (vco2 > 0) {
        rerBuffer.push(vco2 / vo2);
    }
}

bool MetabolicCalculator::detectVT1(ThresholdData& vt1Data) {
    // Simplified VT1 detection using ventilatory equivalent method
    // VT1 occurs where VE/VO2 begins to increase without increase in VE/VCO2
    
    if (vo2Buffer.size() < 30) return false; // Need enough data
    
    // Calculate ventilatory equivalents over recent data
    float veVO2_recent = 0, veVO2_earlier = 0;
    int recentCount = 0, earlierCount = 0;
    
    for (size_t i = vo2Buffer.size() - 10; i < vo2Buffer.size(); i++) {
        if (vo2Buffer[i] > 0) {
            veVO2_recent += ventilationBuffer[i] / vo2Buffer[i];
            recentCount++;
        }
    }
    
    for (size_t i = vo2Buffer.size() - 30; i < vo2Buffer.size() - 10; i++) {
        if (vo2Buffer[i] > 0) {
            veVO2_earlier += ventilationBuffer[i] / vo2Buffer[i];
            earlierCount++;
        }
    }
    
    if (recentCount > 0 && earlierCount > 0) {
        veVO2_recent /= recentCount;
        veVO2_earlier /= earlierCount;
        
        // Check for systematic increase in VE/VO2
        if (veVO2_recent > veVO2_earlier * 1.1f) { // 10% increase
            vt1Data.intensity = 70.0f; // Typical VT1 at ~70% VO2max
            vt1Data.isValid = true;
            return true;
        }
    }
    
    return false;
}

bool MetabolicCalculator::detectVT2(ThresholdData& vt2Data) {
    // Simplified VT2 detection - both VE/VO2 and VE/VCO2 increase
    
    if (vo2Buffer.size() < 30 || !hasCO2Sensor) return false;
    
    // This would implement actual VT2 detection algorithm
    // For now, return false as placeholder
    
    vt2Data.intensity = 85.0f; // Typical VT2 at ~85% VO2max
    vt2Data.isValid = false;
    return false;
}

bool MetabolicCalculator::validateMetabolicData(const MetabolicData& data) {
    return (data.vo2 >= MIN_VO2 && data.vo2 <= MAX_VO2) &&
           (data.vco2 >= MIN_VO2 * MIN_RER && data.vco2 <= MAX_VO2 * MAX_RER) &&
           (data.rer >= MIN_RER && data.rer <= MAX_RER) &&
           (data.ventilationRate >= MIN_VENTILATION && data.ventilationRate <= MAX_VENTILATION) &&
           (data.dataQuality > 0.5f);
}

float MetabolicCalculator::assessDataQuality(const SensorManager::SensorData& input) {
    // Assess overall data quality based on sensor validity and signal characteristics
    int validSensors = 0;
    int totalSensors = 0;
    
    for (int i = 0; i < 8; i++) {
        if (input.sensorValid[i]) validSensors++;
        totalSensors++;
    }
    
    float sensorQuality = totalSensors > 0 ? (float)validSensors / totalSensors : 0.0f;
    
    // Factor in signal quality from sensor manager
    float signalQuality = input.signalQuality;
    
    // Combine metrics
    return (sensorQuality + signalQuality) / 2.0f;
}

// Stub implementations for remaining methods
float MetabolicCalculator::calculateVO2Beaver(float ventilation, float o2Percent, float co2Percent) {
    // Placeholder - would implement Beaver method
    return calculateVO2Wasserman(ventilation, o2Percent);
}

float MetabolicCalculator::calculateVO2VSlope(float vco2, float vo2) {
    // Placeholder - would implement V-slope method
    return vo2;
}

void MetabolicCalculator::resetThresholdDetection() {
    thresholdDetectionActive = false;
    baselineVO2 = 0.0f;
    peakVO2 = 0.0f;
    detectionStartTime = 0;
}

const char* MetabolicCalculator::getIntensityZone(float vo2, float vo2Max) {
    float percentage = (vo2 / vo2Max) * 100.0f;
    
    if (percentage < 50) return "Active Recovery";
    else if (percentage < 60) return "Aerobic Base";
    else if (percentage < 70) return "Aerobic";
    else if (percentage < 80) return "Lactate Threshold";
    else if (percentage < 90) return "VO2max";
    else return "Anaerobic Power";
}