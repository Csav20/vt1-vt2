# VO2Smart - Sistema Profesional de Análisis Respiratorio

<div align="center">

![VO2Smart Dashboard](https://github.com/user-attachments/assets/4dd2c41a-0853-4d22-90fe-db90f2b06366)

**Sistema avanzado de medición de consumo de oxígeno (VO2Max) y análisis metabólico**

[![Patente](https://img.shields.io/badge/Patente-2024024875-blue.svg)](https://patentscope.wipo.int)
[![Versión](https://img.shields.io/badge/Versión-3.0-green.svg)]()
[![ESP32](https://img.shields.io/badge/Hardware-ESP32_TTGO-red.svg)]()
[![Licencia](https://img.shields.io/badge/Licencia-Propietaria-orange.svg)]()

</div>

---

## 📋 Tabla de Contenidos

- [Descripción General](#-descripción-general)
- [Características Principales](#-características-principales)
- [Tecnologías Utilizadas](#-tecnologías-utilizadas)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Interfaz Web (index.html)](#-interfaz-web-indexhtml)
- [Configuración y Uso](#-configuración-y-uso)
- [API y Comunicación](#-api-y-comunicación)
- [Fórmulas y Cálculos](#-fórmulas-y-cálculos)
- [Validación de Datos](#-validación-de-datos)
- [Seguridad y Thread Safety](#-seguridad-y-thread-safety)
- [Troubleshooting](#-troubleshooting)
- [Contribuciones](#-contribuciones)
- [Licencia y Patente](#-licencia-y-patente)

---

## 🎯 Descripción General

**VO2Smart** es un sistema profesional de análisis respiratorio diseñado para medir con precisión el consumo de oxígeno (VO2), producción de CO2 (VCO2) y otros parámetros metabólicos críticos. El sistema está orientado a aplicaciones médicas, de investigación y entrenamiento deportivo de alto rendimiento.

### Inventor
**Claudio Abarca** - Patente Internacional 2024024875

### Optimización
**Csav20** - Mejoras en validación de datos, thread safety y performance (v3.0)

---

## ✨ Características Principales

### 🛡️ Sistema de Validación Integral
- **Validación fisiológica en tiempo real** de todos los sensores
- **Rangos de seguridad** definidos para cada parámetro
- **Detección de anomalías** con alertas automáticas
- **Recuperación progresiva** ante fallos de sensores

### 🔒 Thread Safety con FreeRTOS
- **Mutexes especializados** para diferentes categorías de datos
- **Timeouts configurables** para prevenir deadlocks
- **Mecanismos de fallback** no bloqueantes
- **Gestión automática de recursos**

### 💾 Integridad de Datos EEPROM
- **Checksums CRC32** con validación hardware
- **Detección automática de corrupción**
- **Versionado de configuración** para compatibilidad
- **Recuperación a valores por defecto** cuando sea necesario

### 📊 Interfaz Web Profesional
- **Dashboard en tiempo real** con Chart.js
- **Diseño responsive** adaptable a cualquier dispositivo
- **Múltiples vistas** organizadas por pestañas
- **Documentación técnica** integrada

### ⚡ Optimizaciones de Performance
- **Filtros Kalman optimizados** para ESP32 FPU
- **Funciones inline** para cálculos críticos
- **Parámetros adaptativos** según fase operacional
- **Estructuras de datos eficientes**

---

## 🔧 Tecnologías Utilizadas

### Hardware
- **Microcontrolador**: ESP32 TTGO T-Display (240x135 px)
- **Sensor O2**: DFRobot Oxygen Sensor (galvánico)
- **Sensor CO2**: SCD30 (NDIR)
- **Sensor Presión**: Omron D6F-PH (diferencial)
- **Sensor Ambiental**: BMP280 (temperatura y presión)
- **Conectividad**: Bluetooth Low Energy (BLE)

### Software
- **Firmware**: Arduino Framework para ESP32
- **RTOS**: FreeRTOS (thread management)
- **Interfaz**: HTML5 + CSS3 + JavaScript
- **Gráficos**: Chart.js 4.4.0
- **Iconos**: Font Awesome 6.4.0
- **Almacenamiento**: EEPROM con CRC32

### Librerías ESP32
```cpp
#include <Arduino.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include "DFRobot_OxygenSensor.h"
#include "SCD30.h"
#include "Omron_D6FPH.h"
#include "Adafruit_BMP280.h"
```

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                     VO2Smart System                          │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   Sensores  │───▶│  Validación │───▶│   Filtros   │     │
│  │   Físicos   │    │  Fisiológica│    │   Kalman    │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                   │                    │           │
│         ▼                   ▼                    ▼           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │            Thread-Safe Data Management               │   │
│  │  (Mutexes: Sensor, Settings, ErrorLog, Calculation) │   │
│  └─────────────────────────────────────────────────────┘   │
│         │                                        │           │
│         ▼                                        ▼           │
│  ┌─────────────┐                        ┌─────────────┐    │
│  │   Cálculos  │                        │   EEPROM    │    │
│  │ Metabólicos │                        │  + CRC32    │    │
│  └─────────────┘                        └─────────────┘    │
│         │                                                    │
│         ▼                                                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Output Interfaces                       │   │
│  │    ┌──────────┐  ┌──────────┐  ┌──────────┐        │   │
│  │    │ Display  │  │   BLE    │  │  WebAPI  │        │   │
│  │    │  TTGO    │  │Bluetooth │  │  Server  │        │   │
│  │    └──────────┘  └──────────┘  └──────────┘        │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 🌐 Interfaz Web (index.html)

### Descripción
La interfaz web profesional proporciona visualización en tiempo real de todos los parámetros del sistema VO2Smart, con capacidades de configuración, calibración y diagnóstico.

### Secciones Principales

#### 1. **Header**
- Logo y versión del sistema
- Estado de conexión BLE
- Estado operacional del sistema

#### 2. **Dashboard Principal**
Organizado en un grid responsive con 4 paneles:

**Panel de Sensores en Tiempo Real**
- Oxígeno (O2): 8-25%
- Dióxido de Carbono (CO2): 200-50,000 ppm
- Presión Diferencial: -50 a 5,000 Pa
- Temperatura: -10 a 60°C
- Presión Atmosférica: kPa

**Panel de Consumo de Oxígeno**
- Gráfico en tiempo real de VO2
- Visualización de últimos 60 segundos
- Actualización a 1 Hz

**Panel de Datos Metabólicos**
- VO2 (ml/min)
- VO2 Max (ml/kg/min)
- VCO2 (ml/min)
- RER (Respiratory Exchange Ratio)
- Gasto Energético (kcal/min)
- Ventilación VE (L/min)

**Panel de Flujo Respiratorio**
- Gráfico en tiempo real del flujo
- Medición basada en Venturi
- Correcciones BTPS/STPD

#### 3. **Panel de Configuración**
**Información del Sistema**
- Versión de firmware
- Tipo de dispositivo
- Nivel de batería
- Tiempo activo
- Diámetro Venturi
- Peso del usuario

**Calibración y Configuración**
- Calibración de O2
- Calibración de flujo con jeringa
- Guardado de configuración
- Restablecimiento de valores

#### 4. **Alertas y Diagnóstico**
**Tab de Alertas**
- Alertas de seguridad en tiempo real
- Notificaciones de estado del sistema

**Tab de Registro de Errores**
- Buffer circular de 20 entradas
- Timestamp de cada error
- Descripción detallada

**Tab de Validación**
- Estado de validación por sensor
- Integridad de EEPROM
- Checksums CRC32

#### 5. **Documentación Técnica**
**Tab de Fórmulas**
- Ecuación de Venturi (ISO 5167)
- Cálculo de VO2
- Densidad del aire
- Correcciones BTPS/STPD

**Tab de Rangos de Validación**
- Tabla completa de rangos fisiológicos
- Umbrales críticos
- Valores normales de operación

**Tab Acerca de**
- Información del sistema
- Características principales
- Datos de patente e inventor

### Tecnologías Frontend

```html
<!-- Chart.js para visualización -->
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>

<!-- Font Awesome para iconos -->
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
```

### Personalización CSS
```css
:root {
    --primary-color: #2c3e50;
    --secondary-color: #3498db;
    --success-color: #27ae60;
    --warning-color: #f39c12;
    --danger-color: #e74c3c;
}
```

---

## ⚙️ Configuración y Uso

### 1. Preparación del ESP32

```cpp
// En el sketch principal (revisar)
#define DIAMETER 20  // Diámetro Venturi en mm
#define VERBOSE      // Logging detallado
```

### 2. Servidor Web en ESP32

Para servir el `index.html` desde el ESP32:

```cpp
#include <WebServer.h>
#include <SPIFFS.h>

WebServer server(80);

void setup() {
    // ... inicialización de sensores ...
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Error montando SPIFFS");
        return;
    }
    
    // Servir index.html
    server.on("/", HTTP_GET, []() {
        File file = SPIFFS.open("/index.html", "r");
        server.streamFile(file, "text/html");
        file.close();
    });
    
    // API endpoint para datos en tiempo real
    server.on("/api/vo2", HTTP_GET, []() {
        String json = createDataJSON();
        server.send(200, "application/json", json);
    });
    
    server.begin();
}

void loop() {
    server.handleClient();
    // ... resto del código ...
}
```

### 3. Subir index.html a SPIFFS

```bash
# Usando PlatformIO
pio run --target uploadfs

# O usando Arduino IDE
# Tools -> ESP32 Sketch Data Upload
```

### 4. Acceso a la Interfaz Web

Una vez el ESP32 esté en funcionamiento:

```
http://192.168.4.1/        # En modo AP
http://[IP_DEL_ESP32]/     # En modo Station
```

---

## 📡 API y Comunicación

### Endpoints REST

#### GET /api/vo2
Devuelve todos los datos del sistema en formato JSON:

```json
{
    "o2": 20.93,
    "co2": 400,
    "pressure": 50.0,
    "temperature": 25.0,
    "atmPressure": 101.3,
    "vo2": 250.5,
    "vo2max": 45.5,
    "vco2": 200.3,
    "rer": 0.85,
    "energyExpenditure": 4.8,
    "ventilation": 8.5,
    "flow": 8.5,
    "battery": 85,
    "timestamp": 1234567890
}
```

#### POST /api/calibrate/o2
Inicia calibración de oxígeno:

```json
{
    "reference": 20.93
}
```

#### POST /api/calibrate/flow
Inicia calibración de flujo con jeringa:

```json
{
    "volume": 1.0
}
```

#### POST /api/settings
Guarda configuración del usuario:

```json
{
    "weight": 70,
    "venturiDiameter": 20
}
```

#### GET /api/errors
Obtiene registro de errores:

```json
{
    "errors": [
        {
            "timestamp": 1234567890,
            "sensor": "O2",
            "value": 5.2,
            "reason": "Fuera de rango fisiológico"
        }
    ]
}
```

### Comunicación BLE

**Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

**Características**:
- **VO2 Data**: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- **Sensor Data**: `beb5483e-36e1-4688-b7f5-ea07361b26a9`
- **Control**: `beb5483e-36e1-4688-b7f5-ea07361b26aa`

---

## 📐 Fórmulas y Cálculos

### 1. Flujo Venturi (ISO 5167)

```
Q = A₂ × √(2ΔP / ρ × (1/(A₂/A₁)² - 1))

Donde:
Q  = Flujo volumétrico (m³/s)
A₂ = Área garganta Venturi (m²)
A₁ = Área entrada (m²)
ΔP = Presión diferencial (Pa)
ρ  = Densidad del aire (kg/m³)
```

**Implementación**:
```cpp
float venturiFlowRate = throat_area * sqrt(
    (2.0 * diff_pressure) / 
    (air_density * (1.0 / pow(beta, 2) - 1.0))
);
```

### 2. Consumo de Oxígeno (VO2)

```
VO₂ = VE × (FiO₂ - FeO₂) × ρ_correction

Donde:
VE    = Ventilación por minuto (L/min)
FiO₂  = Fracción inspirada de O₂ (%)
FeO₂  = Fracción espirada de O₂ (%)
ρ_correction = Factor de corrección de densidad
```

**Implementación**:
```cpp
float vo2 = ventilation * (fiO2 - feO2) * density_correction;
```

### 3. Densidad del Aire (Ley de Gases Ideales)

```
ρ = P / (R_específico × T)

Donde:
ρ = Densidad del aire (kg/m³)
P = Presión atmosférica (Pa)
R = Constante específica aire (287.05 J/kg·K)
T = Temperatura absoluta (K)
```

**Implementación**:
```cpp
float air_density = atm_pressure / (287.05 * (temperature + 273.15));
```

### 4. Corrección BTPS (Body Temperature Pressure Saturated)

```
Factor_BTPS = (273 + T_body) / (273 + T_ambient) × (P_ambient - P_H2O) / (P_ambient - 47)

Donde:
T_body = 37°C (temperatura corporal)
T_ambient = Temperatura ambiente (°C)
P_ambient = Presión ambiente (mmHg)
P_H2O = Presión vapor agua ambiente
47 mmHg = Presión vapor agua a 37°C
```

### 5. Corrección STPD (Standard Temperature Pressure Dry)

```
Factor_STPD = 273 / (273 + T_ambient) × P_ambient / 760

Donde:
273 K = Temperatura estándar (0°C)
760 mmHg = Presión estándar
```

### 6. Ratio de Intercambio Respiratorio (RER)

```
RER = VCO₂ / VO₂

Valores típicos:
0.7 = Metabolismo de grasas
0.85 = Metabolismo mixto
1.0 = Metabolismo de carbohidratos
>1.0 = Umbral anaeróbico
```

### 7. Gasto Energético

```
EE = VO₂ × (3.815 + 1.232 × RER)

Donde:
EE = Gasto energético (kcal/min)
VO₂ = Consumo de oxígeno (L/min)
RER = Ratio intercambio respiratorio
```

---

## ✅ Validación de Datos

### Rangos Fisiológicos

| Sensor | Rango Normal | Umbral Crítico | Acción |
|--------|--------------|----------------|--------|
| **O₂** | 8% - 25% | < 10% | Alerta |
| **CO₂** | 200 - 50,000 ppm | > 40,000 ppm | Emergencia |
| **Presión** | -50 a 5,000 Pa | N/A | - |
| **Temperatura** | -10°C a 60°C | < 0°C o > 50°C | Advertencia |

### Sistema de Validación

```cpp
bool validateSensorData(float value, float min, float max, const char* sensorName) {
    if (isnan(value) || isinf(value)) {
        logError(sensorName, value, "Valor inválido (NaN/Inf)");
        return false;
    }
    
    if (value < min || value > max) {
        logError(sensorName, value, "Fuera de rango fisiológico");
        return false;
    }
    
    return true;
}
```

### Estrategia de Recuperación Progresiva

1. **Interpolación**: Usar valores previos válidos
2. **Reset de Sensor**: Reinicializar sensor específico
3. **Valores por Defecto**: Usar valores seguros predeterminados
4. **Alerta al Usuario**: Notificar condición anómala

---

## 🔐 Seguridad y Thread Safety

### Sistema de Mutex FreeRTOS

```cpp
// Declaración de mutexes
SemaphoreHandle_t sensorDataMutex = NULL;
SemaphoreHandle_t settingsMutex = NULL;
SemaphoreHandle_t errorLogMutex = NULL;
SemaphoreHandle_t calculationMutex = NULL;

// Inicialización en setup()
sensorDataMutex = xSemaphoreCreateMutex();
settingsMutex = xSemaphoreCreateMutex();
errorLogMutex = xSemaphoreCreateMutex();
calculationMutex = xSemaphoreCreateMutex();
```

### Uso de Mutex con Timeout

```cpp
if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // Acceso seguro a datos compartidos
    float o2 = readO2Sensor();
    
    xSemaphoreGive(sensorDataMutex);
} else {
    // Timeout - usar mecanismo de fallback
    Serial.println("Mutex timeout - usando último valor válido");
}
```

### Integridad EEPROM con CRC32

```cpp
uint32_t calculateEEPROMChecksum(void* data, size_t size) {
    uint32_t crc = 0xFFFFFFFF;
    uint8_t* ptr = (uint8_t*)data;
    
    for (size_t i = 0; i < size; i++) {
        crc ^= ptr[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    
    return ~crc;
}

bool verifyEEPROMIntegrity() {
    uint32_t stored_checksum = EEPROM.readUInt(CHECKSUM_ADDRESS);
    uint32_t calculated_checksum = calculateEEPROMChecksum(&settings, sizeof(settings));
    
    return (stored_checksum == calculated_checksum);
}
```

---

## 🐛 Troubleshooting

### Problemas Comunes

#### 1. Sensores no responden

**Síntomas**: Valores NaN o cero constante

**Soluciones**:
```cpp
// Verificar conexiones I2C
Wire.beginTransmission(SENSOR_ADDRESS);
if (Wire.endTransmission() != 0) {
    Serial.println("Sensor no detectado en I2C");
}

// Verificar alimentación
// Verificar pines SDA/SCL correctos (TTGO: 21/22)
```

#### 2. Lecturas erráticas

**Síntomas**: Valores fluctúan excesivamente

**Soluciones**:
- Ajustar parámetros del filtro Kalman
- Verificar estabilidad ambiental
- Calibrar sensores
- Revisar cableado y conexiones

#### 3. Dashboard no conecta

**Síntomas**: Estado "Desconectado" permanente

**Soluciones**:
```javascript
// Verificar URL del API
const API_URL = 'http://192.168.4.1/api/vo2';  // Ajustar IP

// Verificar CORS en ESP32
server.sendHeader("Access-Control-Allow-Origin", "*");
```

#### 4. Errores de CRC32

**Síntomas**: "Checksum inválido" en EEPROM

**Soluciones**:
```cpp
// Restablecer EEPROM
void resetEEPROM() {
    loadDefaultSettings();
    saveSettingsWithChecksum();
}
```

### Logs de Diagnóstico

Activar logging verbose:
```cpp
#define VERBOSE  // En el header del sketch
```

Ver logs en Serial Monitor (115200 baud):
```
[INFO] Sistema iniciado
[SENSOR] O2: 20.93%
[SENSOR] CO2: 400 ppm
[CALC] VO2: 250.5 ml/min
[WARN] Presión fuera de rango: -51 Pa
[ERROR] Mutex timeout en sensorDataMutex
```

---

## 👥 Contribuciones

Este proyecto está protegido por **Patente Internacional 2024024875**.

Para consultas sobre colaboraciones o uso comercial, contactar a:
- **Claudio Abarca** (Inventor)
- **Csav20** (Optimización v3.0)

---

## 📄 Licencia y Patente

### Patente
**Número**: 2024024875  
**Titular**: Claudio Abarca  
**Año**: 2024  
**Jurisdicción**: Internacional

### Derechos de Autor
© 2024-2025 Claudio Abarca. Todos los derechos reservados.

**Optimización v3.0**: Csav20

### Uso Permitido
- ✅ Uso personal y educativo
- ✅ Investigación académica (con cita apropiada)
- ❌ Uso comercial sin autorización
- ❌ Redistribución sin permiso
- ❌ Modificación de patente o marcas

### Cita Recomendada
```
Abarca, C. (2024). VO2Smart: Sistema Profesional de Análisis Respiratorio.
Patente 2024024875. Optimización v3.0 por Csav20.
```

---

## 📞 Contacto y Soporte

Para consultas técnicas, reportes de bugs o solicitudes de características:

- **Issues**: [GitHub Issues](https://github.com/Csav20/vt1-vt2/issues)
- **Documentación**: Este README y comentarios en código
- **Email**: [Contactar a través de GitHub]

---

<div align="center">

**VO2Smart** - Transformando el análisis respiratorio profesional

*Powered by ESP32 | Patente 2024024875 | Optimizado por Csav20*

</div>
