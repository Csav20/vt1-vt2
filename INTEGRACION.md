# Guía Rápida de Integración - index.html con ESP32

## 📋 Resumen
Este documento explica cómo integrar el archivo `index.html` profesional con el sistema VO2Smart en ESP32.

## 🚀 Opción 1: Servidor Web ESP32 con SPIFFS (Recomendado)

### Paso 1: Instalar Herramientas SPIFFS

**Para PlatformIO:**
```ini
; platformio.ini
[env:ttgo-t-display]
platform = espressif32
board = esp32dev
framework = arduino
board_build.filesystem = spiffs
monitor_speed = 115200
lib_deps = 
    ESP Async WebServer
    ESPAsyncTCP
```

**Para Arduino IDE:**
1. Descargar [ESP32FS Plugin](https://github.com/me-no-dev/arduino-esp32fs-plugin)
2. Instalar en: `<Arduino>/tools/ESP32FS/tool/esp32fs.jar`

### Paso 2: Preparar Estructura de Archivos

```
tu_proyecto/
├── src/
│   └── main.cpp (o revisar.ino)
└── data/
    ├── index.html
    └── (otros archivos web si necesarios)
```

Copiar `index.html` a la carpeta `data/`

### Paso 3: Código ESP32 para Servidor Web

```cpp
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// Credenciales WiFi
const char* ssid = "TU_WIFI";
const char* password = "TU_PASSWORD";

// Servidor en puerto 80
AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);
    
    // Inicializar SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Error montando SPIFFS");
        return;
    }
    
    // Conectar WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    // Ruta principal - servir index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    
    // API endpoint - datos en tiempo real
    server.on("/api/vo2", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Crear JSON con datos actuales
        String json = "{";
        json += "\"o2\":" + String(currentO2, 2) + ",";
        json += "\"co2\":" + String(currentCO2, 1) + ",";
        json += "\"pressure\":" + String(currentPressure, 1) + ",";
        json += "\"temperature\":" + String(currentTemp, 1) + ",";
        json += "\"atmPressure\":" + String(currentAtmPressure, 1) + ",";
        json += "\"vo2\":" + String(calculatedVO2, 1) + ",";
        json += "\"vo2max\":" + String(calculatedVO2Max, 1) + ",";
        json += "\"vco2\":" + String(calculatedVCO2, 1) + ",";
        json += "\"rer\":" + String(calculatedRER, 2) + ",";
        json += "\"energyExpenditure\":" + String(energyExpenditure, 1) + ",";
        json += "\"ventilation\":" + String(ventilation, 1) + ",";
        json += "\"flow\":" + String(flowRate, 1) + ",";
        json += "\"battery\":" + String(batteryLevel) + ",";
        json += "\"timestamp\":" + String(millis() / 1000);
        json += "}";
        
        request->send(200, "application/json", json);
    });
    
    // API endpoint - calibración O2
    server.on("/api/calibrate/o2", HTTP_POST, [](AsyncWebServerRequest *request) {
        // Implementar calibración
        fnCalO2();
        request->send(200, "application/json", "{\"status\":\"success\"}");
    });
    
    // API endpoint - calibración flujo
    server.on("/api/calibrate/flow", HTTP_POST, [](AsyncWebServerRequest *request) {
        // Implementar calibración de flujo
        performSyringeCalibration();
        request->send(200, "application/json", "{\"status\":\"success\"}");
    });
    
    // API endpoint - guardar configuración
    server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
        // Extraer peso del usuario del request
        if (request->hasParam("weight", true)) {
            float weight = request->getParam("weight", true)->value().toFloat();
            // Guardar en EEPROM
            saveSettings();
        }
        request->send(200, "application/json", "{\"status\":\"saved\"}");
    });
    
    // Iniciar servidor
    server.begin();
    Serial.println("Servidor HTTP iniciado");
}

void loop() {
    // Tu código existente de lectura de sensores y cálculos
    readSensors();
    calculateMetrics();
    delay(1000);  // Actualizar cada segundo
}
```

### Paso 4: Subir Archivos

**PlatformIO:**
```bash
pio run --target uploadfs
pio run --target upload
```

**Arduino IDE:**
1. Tools → ESP32 Sketch Data Upload (para SPIFFS)
2. Upload (para código)

### Paso 5: Acceder a la Interfaz

```
http://[IP_DEL_ESP32]/
```

La IP se muestra en el Serial Monitor al iniciar.

---

## 🚀 Opción 2: Modo Access Point (Sin WiFi Externo)

Si no quieres depender de una red WiFi existente:

```cpp
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

const char* ap_ssid = "VO2Smart";
const char* ap_password = "vo2smart123";

AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Error montando SPIFFS");
        return;
    }
    
    // Crear Access Point
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress IP = WiFi.softAPIP();
    
    Serial.println("Access Point creado");
    Serial.print("IP: ");
    Serial.println(IP);  // Típicamente 192.168.4.1
    
    // Configurar rutas igual que antes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    
    // ... resto de endpoints API ...
    
    server.begin();
}
```

**Acceso:**
1. Conectarse a WiFi "VO2Smart" (password: vo2smart123)
2. Abrir navegador en: `http://192.168.4.1/`

---

## 🚀 Opción 3: Sin SPIFFS - HTML Embebido (Para Testing Rápido)

Si no quieres usar SPIFFS:

```cpp
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VO2Smart</title>
    <!-- Pegar aquí el contenido de index.html -->
</head>
<body>
    <!-- ... -->
</body>
</html>
)rawliteral";

void setup() {
    // ...
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });
}
```

**Nota:** Esta opción tiene límites de memoria. Para el `index.html` completo (47KB), usar SPIFFS.

---

## 📝 Modificar index.html para Producción

### En el archivo index.html, localiza:

```javascript
async function fetchData() {
    try {
        // ⚠️ PRODUCCIÓN: Descomentar las siguientes líneas para usar API real del ESP32
        // const response = await fetch('/api/vo2');
        // const data = await response.json();
        
        // 🧪 DESARROLLO: Usando datos simulados para pruebas
        const data = await simulateServerData();
```

### Cambiar a:

```javascript
async function fetchData() {
    try {
        // Usar API real del ESP32
        const response = await fetch('/api/vo2');
        const data = await response.json();
```

### Eliminar o comentar la función `simulateServerData()` si no la necesitas.

---

## 🧪 Testing

### 1. Verificar SPIFFS
```cpp
void listSPIFFS() {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file) {
        Serial.print("FILE: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
}
```

### 2. Monitor Serial
Deberías ver:
```
WiFi conectado!
IP: 192.168.1.100
Servidor HTTP iniciado
FILE: /index.html
```

### 3. Abrir Navegador
```
http://192.168.1.100/
```

Deberías ver el dashboard completo.

---

## ⚠️ Solución de Problemas

### index.html no se carga
- ✅ Verificar que SPIFFS se montó correctamente
- ✅ Verificar que index.html está en carpeta `data/`
- ✅ Ejecutar `uploadfs` antes de `upload`

### API no responde
- ✅ Verificar que los endpoints están definidos en el código
- ✅ Abrir DevTools del navegador (F12) → Network tab
- ✅ Verificar que la URL es correcta: `/api/vo2`

### Gráficos no aparecen
- ✅ Verificar conexión a internet (Chart.js desde CDN)
- ✅ O descargar Chart.js localmente y servirlo desde SPIFFS
- ✅ Revisar consola del navegador por errores JavaScript

### CORS Errors
Agregar headers en ESP32:
```cpp
server.on("/api/vo2", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
});
```

---

## 📚 Recursos Adicionales

- **Documentación ESP32 WebServer**: https://github.com/me-no-dev/ESPAsyncWebServer
- **SPIFFS**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spiffs.html
- **Chart.js**: https://www.chartjs.org/docs/

---

## ✅ Checklist de Integración

- [ ] SPIFFS instalado y configurado
- [ ] `index.html` copiado a carpeta `data/`
- [ ] Código del servidor web agregado al sketch
- [ ] Endpoints API implementados
- [ ] Archivos subidos (uploadfs + upload)
- [ ] WiFi conectado o AP creado
- [ ] Dashboard accesible desde navegador
- [ ] Datos reales mostrados (no simulados)
- [ ] Calibración funcional
- [ ] Guardado de configuración funcional

---

**¡Éxito! Tu sistema VO2Smart ahora tiene una interfaz web profesional.**

Para cualquier duda, consulta el README.md completo o los comentarios en el código.
