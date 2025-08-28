# VO2Smart - Advanced ESP32 Metabolic Monitoring System

## Overview

VO2Smart is a comprehensive ESP32-based system for real-time measurement and analysis of metabolic parameters including VO2, VCO2, and ventilatory thresholds (VT1/VT2). This system has been completely restructured and optimized for professional use in sports science and metabolic research.

## Key Features

### 🔬 **Advanced Metabolic Analysis**
- Real-time VO2/VCO2 measurement with multiple calculation methods
- Automatic ventilatory threshold detection (VT1/VT2)
- Substrate utilization analysis (carbohydrate/fat oxidation)
- Energy expenditure calculation using Weir equation
- Metabolic efficiency tracking

### 📊 **Dynamic Visualization**
- Real-time graphs for VO2/VCO2 trends
- Multi-screen interface with specialized views
- Threshold detection visualization
- System status and sensor health indicators
- Responsive grid-based layout

### 🔧 **Optimized Performance**
- Dual-core ESP32 architecture utilization
- Pre-calculated constants for mathematical operations
- Advanced filtering (Kalman, Moving Average, Quantum-enhanced)
- Efficient memory management with circular buffers
- Real-time performance monitoring

### 📡 **Robust Connectivity**
- Dual connectivity (BLE + WiFi) with automatic failover
- Exponential backoff reconnection strategy
- Comprehensive error handling and recovery
- Web-based data access and monitoring
- Real-time data streaming

### 🛡️ **Enhanced Reliability**
- Comprehensive sensor validation and error detection
- System health monitoring with watchdog
- Automatic sensor recovery mechanisms
- Detailed error reporting and diagnostics
- Graceful degradation under fault conditions

## Hardware Requirements

### Core Components
- **ESP32 Development Board** (ESP32-WROOM-32 recommended)
- **TFT Display** (240x320 pixels, SPI interface)
- **Metabolic Sensors**:
  - MAX30102: Heart rate and SpO2
  - MAX30105: NIRS and metabolite detection
  - VL53L0X: Time-of-flight distance sensor
  - MLX90614: Non-contact temperature sensor
  - BMP280: Environmental pressure/temperature
  - QMI8658: 6-axis IMU (replaces MPU6050)

### Pin Configuration
```
I2C Bus:     SDA = GPIO 21, SCL = GPIO 22
TFT Display: CS = GPIO 5, DC = GPIO 2, RST = GPIO 4
Buttons:     Power = GPIO 0, Mode = GPIO 15
Status:      LED = GPIO 2, Buzzer = GPIO 13
```

## Software Architecture

### Modular Design
```
┌─────────────────┐    ┌─────────────────┐
│   CoreManager   │────│  SensorManager  │
│  (Task Control) │    │ (Data Acquisition)│
└─────────────────┘    └─────────────────┘
         │                        │
┌─────────────────┐    ┌─────────────────┐
│ DisplayManager  │────│MetabolicCalc    │
│ (Visualization) │    │ (Analysis)      │
└─────────────────┘    └─────────────────┘
         │                        │
┌─────────────────┐    ┌─────────────────┐
│ConnectivityMgr  │────│   Utilities     │
│(Communication)  │    │ (Filters/Buffers)│
└─────────────────┘    └─────────────────┘
```

### Core Components

#### 1. **CoreManager**
- Dual-core task distribution and management
- System health monitoring with watchdog
- Error handling and recovery coordination
- Performance monitoring and optimization

#### 2. **SensorManager**
- Multi-sensor initialization and validation
- Advanced filtering and noise reduction
- Automatic calibration and error detection
- Optimized data acquisition with pre-calculated constants

#### 3. **MetabolicCalculator**
- Multiple VO2/VCO2 calculation methods (Wasserman, Beaver)
- Real-time threshold detection algorithms
- Substrate utilization analysis
- Data quality assessment and validation

#### 4. **DisplayManager**
- Multi-screen interface with dynamic content
- Real-time graph rendering with optimization
- Status indication and error messaging
- Touch interface support (configurable)

#### 5. **ConnectivityManager**
- Robust BLE and WiFi connectivity
- Automatic reconnection with exponential backoff
- Web server for remote monitoring
- Comprehensive error handling and diagnostics

## Installation and Setup

### Prerequisites
- PlatformIO IDE or Arduino IDE with ESP32 support
- Required libraries (automatically installed via platformio.ini)

### Quick Start
1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd vt1-vt2
   ```

2. **Configure hardware**:
   - Review `include/config.h` for pin assignments
   - Enable/disable sensors based on your hardware setup
   - Adjust I2C addresses if needed

3. **Build and upload**:
   ```bash
   platformio run -t upload
   ```

4. **Monitor operation**:
   ```bash
   platformio device monitor
   ```

### Configuration Options

Edit `include/config.h` to customize:

```cpp
// Enable/disable sensors
#define USE_MAX30102           1
#define USE_MAX30105           1
#define USE_VL53L0X            1
// ... other sensors

// Performance tuning
#define SENSOR_TASK_FREQ_HZ    20
#define UI_TASK_FREQ_HZ        10

// Connectivity options
#define USE_BLE                1
#define USE_WIFI               1
#define DUAL_CONNECTIVITY      1
```

## Usage

### Basic Operation
1. **Power on**: The system automatically initializes all sensors
2. **Calibration**: Follow on-screen prompts for sensor calibration
3. **Monitoring**: Real-time data appears on the display
4. **Data Access**: Connect via BLE or access web interface at device IP

### Serial Commands
- `diag` - Run system diagnostics
- `cal` - Enter calibration mode
- `reset` - Restart the system
- `info` - Display system information

### Screen Navigation
- **Main Screen**: Real-time VO2/VCO2 with graphs
- **Respiratory**: Detailed ventilatory parameters
- **Thresholds**: VT1/VT2 analysis and detection
- **Calorimetry**: Energy expenditure and substrate utilization
- **Environment**: Sensor status and environmental conditions

## Advanced Features

### Calculation Optimizations
- **Pre-calculated Constants**: Venturi area, gas densities, conversion factors
- **Simplified Bernoulli**: Optimized flow calculations
- **BTPS/STPD Corrections**: Automatic temperature/pressure corrections
- **Multiple Methods**: Cross-validation using different calculation approaches

### Filtering and Signal Processing
- **Kalman Filtering**: Adaptive noise reduction for physiological signals
- **Quantum-Enhanced Filtering**: Advanced filtering for rapidly changing metabolic parameters
- **Moving Averages**: Configurable window sizes for different signal types
- **Circular Buffers**: Efficient memory usage for time-series data

### Threshold Detection
- **VT1 Detection**: Ventilatory equivalent method with automated analysis
- **VT2 Detection**: Dual-criteria method for accurate threshold identification
- **Real-time Analysis**: Continuous monitoring with immediate feedback
- **Quality Assessment**: Confidence scoring for threshold detection

## API Reference

### Data Structures
```cpp
// Sensor data structure
struct SensorData {
    float pressure, o2Percent, co2Ppm;
    float ambientTemp, ambientPressure, humidity;
    float irReflectance, redReflectance;
    float acceleration[3], gyroscope[3];
    bool sensorValid[8];
    float signalQuality;
    uint32_t timestamp;
};

// Metabolic analysis results
struct MetabolicData {
    float vo2, vo2Max, vco2, rer;
    float energyExpenditure;
    float carbPercentage, fatPercentage;
    float vt1, vt2;
    bool vt1Detected, vt2Detected;
    float dataQuality;
};
```

### Key Functions
```cpp
// System control
CoreManager::initialize();
CoreManager::createTasks();

// Sensor operations
SensorManager::begin();
SensorManager::readSensors(sensorData);
SensorManager::calibrateO2(20.93f);

// Metabolic calculations
MetabolicCalculator::configure(75.0f, 175.0f, 25);
MetabolicCalculator::calculate(sensorData, metabolicData);

// Display control
DisplayManager::setScreen(MAIN_METRICS);
DisplayManager::showError("Error", "Message");

// Connectivity
ConnectivityManager::sendData(jsonString);
ConnectivityManager::isConnected();
```

## Performance Characteristics

### Task Distribution
- **Core 0**: Sensor reading (20Hz), metabolic calculations (5Hz)
- **Core 1**: Display updates (10Hz), connectivity management

### Memory Usage
- **Heap**: ~180KB typical usage
- **Stack**: Optimized per task (8-12KB each)
- **Buffers**: Efficient circular buffers (1-5KB each)

### Timing Performance
- **Sensor Read**: <5ms typical
- **Calculations**: <10ms per cycle
- **Display Update**: <50ms full refresh
- **BLE Transmission**: <20ms per packet

## Troubleshooting

### Common Issues

1. **Sensor Initialization Failure**
   - Check I2C connections and addresses
   - Verify power supply stability
   - Run diagnostic command: `diag`

2. **Display Problems**
   - Verify SPI pin connections
   - Check TFT library compatibility
   - Adjust brightness settings

3. **Connectivity Issues**
   - Check WiFi credentials in config
   - Verify BLE is enabled on client device
   - Monitor connection status on display

4. **Calculation Errors**
   - Ensure sensors are properly calibrated
   - Check data quality indicators
   - Verify environmental conditions are stable

### Debug Information
- Enable serial debug output in `config.h`
- Monitor task performance with `ENABLE_PERFORMANCE_MON`
- Use memory debugging for heap analysis

## Contributing

### Development Guidelines
1. Follow the modular architecture pattern
2. Maintain separation between hardware abstraction and business logic
3. Add comprehensive error handling for new features
4. Update documentation for API changes
5. Test on real hardware before submitting

### Code Style
- Use descriptive variable and function names
- Add comprehensive comments for complex algorithms
- Follow existing naming conventions
- Maintain const correctness and type safety

## License

This project is released under the MIT License. See LICENSE file for details.

## Acknowledgments

- Based on the original VO2Smart project
- Incorporates industry-standard metabolic calculation methods
- Utilizes established filtering and signal processing techniques
- Designed for compatibility with standard metabolic testing protocols

## Version History

### v2.1.0 (Current)
- Complete modular restructuring
- Advanced threshold detection algorithms
- Optimized dual-core architecture
- Enhanced connectivity with auto-reconnection
- Comprehensive error handling and recovery

### v2.0.0
- Initial ESP32 implementation
- Basic VO2/VCO2 calculations
- Simple display interface
- Basic BLE connectivity

---

For technical support or feature requests, please open an issue in the project repository.