/*
 * IMPORTANT: This file has been replaced with a modular architecture!
 * 
 * The original main.cpp (now main_original.cpp) contained a monolithic
 * implementation mixed with documentation. It has been completely 
 * restructured into professional modules:
 * 
 * NEW STRUCTURE:
 * =============
 * src/main.cpp              - Clean entry point and user interaction
 * src/CoreManager.cpp       - Dual-core task management  
 * src/SensorManager.cpp     - Sensor handling and validation
 * src/MetabolicCalculator.cpp - VO2/VCO2 calculations and thresholds
 * src/DisplayManager.cpp    - Multi-screen UI with real-time graphs
 * src/ConnectivityManager.cpp - BLE/WiFi with auto-reconnection
 * 
 * IMPROVEMENTS:
 * ============
 * ✅ Optimized calculations with pre-calculated constants
 * ✅ Dynamic graphs for VO2/VCO2 and threshold monitoring  
 * ✅ Robust BLE/WiFi with automatic reconnection
 * ✅ Modular code structure for maintainability
 * ✅ Enhanced error detection and user feedback
 * ✅ Dual-core ESP32 optimization
 * ✅ Advanced filtering and signal processing
 * ✅ Comprehensive system health monitoring
 * 
 * USAGE:
 * ======
 * 1. Review include/config.h for hardware configuration
 * 2. Build with: platformio run -t upload
 * 3. Monitor with: platformio device monitor
 * 
 * See README.md for complete documentation.
 * 
 * The actual main entry point is now in src/main.cpp
 */

#error "Please use the new modular structure in src/main.cpp instead of this file"