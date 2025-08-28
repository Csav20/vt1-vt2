#include "ConnectivityManager.h"
#include "config.h"
#include <Arduino.h>

// Static member definitions
ConnectivityManager::Config ConnectivityManager::config;
ConnectivityManager::ConnectionState ConnectivityManager::state;
bool ConnectivityManager::initialized = false;
BLEServer* ConnectivityManager::pServer = nullptr;
BLEService* ConnectivityManager::pService = nullptr;
BLECharacteristic* ConnectivityManager::pDataCharacteristic = nullptr;
BLECharacteristic* ConnectivityManager::pControlCharacteristic = nullptr;
BLECharacteristic* ConnectivityManager::pStatusCharacteristic = nullptr;
BLEAdvertising* ConnectivityManager::pAdvertising = nullptr;
WebServer* ConnectivityManager::webServer = nullptr;
char ConnectivityManager::lastError[128] = "";
uint32_t ConnectivityManager::errorCount = 0;
uint32_t ConnectivityManager::lastErrorTime = 0;
uint32_t ConnectivityManager::nextBLEReconnect = 0;
uint32_t ConnectivityManager::nextWiFiReconnect = 0;
uint32_t ConnectivityManager::bleBackoffDelay = MIN_BACKOFF_DELAY;
uint32_t ConnectivityManager::wifiBackoffDelay = MIN_BACKOFF_DELAY;
bool ConnectivityManager::compressionEnabled = false;
uint16_t ConnectivityManager::transmissionRate = 10;
uint32_t ConnectivityManager::lastTransmission = 0;
uint32_t ConnectivityManager::bytesTransmitted = 0;
uint32_t ConnectivityManager::throughputCalculationTime = 0;

// UUID constants
const char* ConnectivityManager::SERVICE_UUID = BLE_SERVICE_UUID;
const char* ConnectivityManager::DATA_CHAR_UUID = BLE_DATA_CHAR_UUID;
const char* ConnectivityManager::CONTROL_CHAR_UUID = BLE_CONTROL_CHAR_UUID;
const char* ConnectivityManager::STATUS_CHAR_UUID = BLE_STATUS_CHAR_UUID;

// BLE Server Callbacks
class ConnectivityManager::ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("BLE Client connected");
        ConnectivityManager::state.bleStatus = CONNECTED;
        ConnectivityManager::state.bleConnectedTime = millis();
        ConnectivityManager::state.bleReconnectCount = 0;
        ConnectivityManager::bleBackoffDelay = MIN_BACKOFF_DELAY;
    }
    
    void onDisconnect(BLEServer* pServer) {
        Serial.println("BLE Client disconnected");
        ConnectivityManager::handleBLEDisconnection();
    }
};

// BLE Characteristic Callbacks
class ConnectivityManager::CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial.printf("BLE Received: %s\n", value.c_str());
        
        // Handle control commands
        if (pCharacteristic == ConnectivityManager::pControlCharacteristic) {
            // Process control commands here
            if (value == "PING") {
                ConnectivityManager::pStatusCharacteristic->setValue("PONG");
                ConnectivityManager::pStatusCharacteristic->notify();
            }
        }
    }
};

void ConnectivityManager::begin(const Config& configParam) {
    config = configParam;
    
    // Initialize connection state
    state.bleStatus = DISCONNECTED;
    state.wifiStatus = DISCONNECTED;
    state.bleConnectedTime = 0;
    state.wifiConnectedTime = 0;
    state.lastBLEAttempt = 0;
    state.lastWiFiAttempt = 0;
    state.bleReconnectCount = 0;
    state.wifiReconnectCount = 0;
    state.wifiSignalStrength = 0;
    state.connectedClients = 0;
    
    Serial.println("Initializing connectivity...");
    
    #if USE_BLE
    if (config.enableBLE) {
        setupBLE();
    }
    #endif
    
    #if USE_WIFI
    if (config.enableWiFi) {
        setupWiFi();
    }
    #endif
    
    initialized = true;
    Serial.println("Connectivity initialization complete");
}

void ConnectivityManager::setupBLE() {
    Serial.println("Setting up BLE...");
    
    BLEDevice::init(config.deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create service
    pService = pServer->createService(SERVICE_UUID);
    
    // Create characteristics
    pDataCharacteristic = pService->createCharacteristic(
        DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pControlCharacteristic = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    
    pStatusCharacteristic = pService->createCharacteristic(
        STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    // Add descriptors
    pDataCharacteristic->addDescriptor(new BLE2902());
    pStatusCharacteristic->addDescriptor(new BLE2902());
    
    // Set callbacks
    pControlCharacteristic->setCallbacks(new CharacteristicCallbacks());
    
    // Start service
    pService->start();
    
    // Start advertising
    startBLEAdvertising();
    
    state.bleStatus = CONNECTING;
    Serial.println("BLE setup complete");
}

void ConnectivityManager::startBLEAdvertising() {
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    
    Serial.println("BLE advertising started");
}

void ConnectivityManager::setupWiFi() {
    Serial.println("Setting up WiFi...");
    
    WiFi.mode(WIFI_STA);
    
    if (connectToWiFi(config.wifiSSID, config.wifiPassword)) {
        setupWebServer();
    } else if (config.fallbackSSID) {
        Serial.println("Trying fallback WiFi...");
        connectToWiFi(config.fallbackSSID, config.fallbackPassword);
    }
}

bool ConnectivityManager::connectToWiFi(const char* ssid, const char* password) {
    state.wifiStatus = CONNECTING;
    state.lastWiFiAttempt = millis();
    
    WiFi.begin(ssid, password);
    
    Serial.printf("Connecting to WiFi: %s\n", ssid);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        state.wifiStatus = CONNECTED;
        state.wifiConnectedTime = millis();
        state.wifiReconnectCount = 0;
        wifiBackoffDelay = MIN_BACKOFF_DELAY;
        
        Serial.println("\nWiFi connected!");
        Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
        
        state.wifiSignalStrength = WiFi.RSSI();
        return true;
    } else {
        state.wifiStatus = FAILED;
        state.wifiReconnectCount++;
        Serial.println("\nWiFi connection failed");
        return false;
    }
}

void ConnectivityManager::setupWebServer() {
    if (webServer) {
        delete webServer;
    }
    
    webServer = new WebServer(config.webServerPort);
    
    // Define routes
    webServer->on("/", handleRoot);
    webServer->on("/data", HTTP_GET, handleData);
    webServer->on("/status", HTTP_GET, handleStatus);
    webServer->on("/config", HTTP_POST, handleConfig);
    webServer->onNotFound(handleNotFound);
    
    // Enable CORS
    webServer->enableCORS(true);
    
    webServer->begin();
    Serial.printf("Web server started on port %d\n", config.webServerPort);
}

void ConnectivityManager::update() {
    if (!initialized) return;
    
    uint32_t currentTime = millis();
    
    // Update connection status
    updateConnectionStatus();
    
    // Handle BLE reconnection
    #if USE_BLE
    if (config.enableBLE && state.bleStatus == DISCONNECTED) {
        if (shouldAttemptReconnection(state.bleStatus, state.lastBLEAttempt, config.reconnectInterval)) {
            reconnectBLE();
        }
    }
    #endif
    
    // Handle WiFi reconnection
    #if USE_WIFI
    if (config.enableWiFi && state.wifiStatus == DISCONNECTED) {
        if (shouldAttemptReconnection(state.wifiStatus, state.lastWiFiAttempt, config.reconnectInterval)) {
            reconnectWiFi();
        }
    }
    #endif
    
    // Update throughput statistics
    if (currentTime - throughputCalculationTime > THROUGHPUT_CALCULATION_INTERVAL) {
        updateThroughputStats(0); // Reset for next interval
        throughputCalculationTime = currentTime;
    }
}

void ConnectivityManager::handleClients() {
    #if USE_WIFI
    if (webServer && state.wifiStatus == CONNECTED) {
        webServer->handleClient();
    }
    #endif
}

bool ConnectivityManager::sendData(const String& jsonData) {
    bool success = false;
    
    // Try BLE first if available
    #if USE_BLE
    if (config.enableBLE && state.bleStatus == CONNECTED) {
        success |= sendDataBLE((uint8_t*)jsonData.c_str(), jsonData.length());
    }
    #endif
    
    // Try WiFi if available
    #if USE_WIFI
    if (config.enableWiFi && state.wifiStatus == CONNECTED) {
        success |= sendDataWiFi(jsonData);
    }
    #endif
    
    if (success) {
        updateThroughputStats(jsonData.length());
        lastTransmission = millis();
    }
    
    return success;
}

bool ConnectivityManager::sendDataBLE(const uint8_t* data, size_t length) {
    if (!pDataCharacteristic || state.bleStatus != CONNECTED) {
        return false;
    }
    
    try {
        // Split large data into chunks if necessary
        size_t maxChunkSize = BLE_MTU_SIZE - 3; // Account for BLE overhead
        size_t offset = 0;
        
        while (offset < length) {
            size_t chunkSize = min(maxChunkSize, length - offset);
            
            pDataCharacteristic->setValue(data + offset, chunkSize);
            pDataCharacteristic->notify();
            
            offset += chunkSize;
            
            if (chunkSize == maxChunkSize) {
                delay(10); // Small delay between chunks
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        logError("BLE", "Failed to send data");
        return false;
    }
}

bool ConnectivityManager::sendDataWiFi(const String& data) {
    // This would typically broadcast to connected web clients
    // For now, just store the data for web requests
    return true;
}

void ConnectivityManager::handleBLEDisconnection() {
    state.bleStatus = DISCONNECTED;
    state.bleReconnectCount++;
    calculateBackoffDelay(bleBackoffDelay, state.bleReconnectCount);
    nextBLEReconnect = millis() + bleBackoffDelay;
    
    logError("BLE", "Connection lost");
}

bool ConnectivityManager::reconnectBLE() {
    if (state.bleStatus == CONNECTING) return false;
    
    Serial.println("Attempting BLE reconnection...");
    state.bleStatus = CONNECTING;
    state.lastBLEAttempt = millis();
    
    try {
        startBLEAdvertising();
        return true;
    } catch (const std::exception& e) {
        state.bleStatus = FAILED;
        logError("BLE", "Reconnection failed");
        return false;
    }
}

bool ConnectivityManager::reconnectWiFi() {
    if (state.wifiStatus == CONNECTING) return false;
    
    Serial.println("Attempting WiFi reconnection...");
    return connectToWiFi(config.wifiSSID, config.wifiPassword);
}

void ConnectivityManager::updateConnectionStatus() {
    // Update BLE status
    #if USE_BLE
    if (config.enableBLE && pServer) {
        uint32_t connectedClients = pServer->getConnectedCount();
        if (connectedClients > 0 && state.bleStatus != CONNECTED) {
            state.bleStatus = CONNECTED;
        } else if (connectedClients == 0 && state.bleStatus == CONNECTED) {
            state.bleStatus = DISCONNECTED;
        }
    }
    #endif
    
    // Update WiFi status
    #if USE_WIFI
    if (config.enableWiFi) {
        if (WiFi.status() == WL_CONNECTED && state.wifiStatus != CONNECTED) {
            state.wifiStatus = CONNECTED;
            state.wifiSignalStrength = WiFi.RSSI();
        } else if (WiFi.status() != WL_CONNECTED && state.wifiStatus == CONNECTED) {
            state.wifiStatus = DISCONNECTED;
            handleWiFiDisconnection();
        }
    }
    #endif
}

void ConnectivityManager::handleWiFiDisconnection() {
    state.wifiReconnectCount++;
    calculateBackoffDelay(wifiBackoffDelay, state.wifiReconnectCount);
    nextWiFiReconnect = millis() + wifiBackoffDelay;
    
    logError("WiFi", "Connection lost");
}

bool ConnectivityManager::shouldAttemptReconnection(ConnectionStatus status, uint32_t lastAttempt, uint32_t interval) {
    return (status == DISCONNECTED || status == FAILED) && 
           (millis() - lastAttempt > interval);
}

void ConnectivityManager::calculateBackoffDelay(uint32_t& backoffDelay, uint8_t attemptCount) {
    // Exponential backoff with jitter
    backoffDelay = MIN_BACKOFF_DELAY * (1 << min(attemptCount, (uint8_t)6)); // Max 64x base delay
    backoffDelay = min(backoffDelay, MAX_BACKOFF_DELAY);
    
    // Add jitter (±25%)
    int32_t jitter = (backoffDelay / 4) * (random(-100, 100) / 100.0f);
    backoffDelay += jitter;
}

void ConnectivityManager::logError(const char* source, const char* message) {
    snprintf(lastError, sizeof(lastError), "%s: %s", source, message);
    errorCount++;
    lastErrorTime = millis();
    
    Serial.printf("Connectivity Error - %s\n", lastError);
}

void ConnectivityManager::updateThroughputStats(size_t bytes) {
    bytesTransmitted += bytes;
}

// Web server handlers
void ConnectivityManager::handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>VO2Smart</title></head><body>";
    html += "<h1>VO2Smart Metabolic Monitor</h1>";
    html += "<p>Device is running and collecting data.</p>";
    html += "<a href='/data'>View Current Data</a><br>";
    html += "<a href='/status'>System Status</a>";
    html += "</body></html>";
    
    webServer->send(200, "text/html", html);
}

void ConnectivityManager::handleData() {
    // Return current sensor data as JSON
    String json = "{";
    json += "\"timestamp\":" + String(millis()) + ",";
    json += "\"vo2\":42.5,";
    json += "\"vco2\":38.1,";
    json += "\"rer\":0.89,";
    json += "\"hr\":145";
    json += "}";
    
    webServer->send(200, "application/json", json);
}

void ConnectivityManager::handleStatus() {
    String json = "{";
    json += "\"ble_status\":" + String(state.bleStatus) + ",";
    json += "\"wifi_status\":" + String(state.wifiStatus) + ",";
    json += "\"wifi_rssi\":" + String(state.wifiSignalStrength) + ",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"free_heap\":" + String(ESP.getFreeHeap());
    json += "}";
    
    webServer->send(200, "application/json", json);
}

void ConnectivityManager::handleConfig() {
    // Handle configuration updates
    webServer->send(200, "text/plain", "Configuration update not implemented");
}

void ConnectivityManager::handleNotFound() {
    webServer->send(404, "text/plain", "Not Found");
}

// Stub implementations for remaining methods
bool ConnectivityManager::isBLEConnected() {
    return state.bleStatus == CONNECTED;
}

bool ConnectivityManager::isWiFiConnected() {
    return state.wifiStatus == CONNECTED;
}

bool ConnectivityManager::isAnyConnectionActive() {
    return isBLEConnected() || isWiFiConnected();
}

ConnectivityManager::ConnectionState ConnectivityManager::getConnectionState() {
    return state;
}

const char* ConnectivityManager::getLastError() {
    return lastError;
}

uint32_t ConnectivityManager::getErrorCount() {
    return errorCount;
}

void ConnectivityManager::clearErrors() {
    errorCount = 0;
    lastError[0] = '\0';
    lastErrorTime = 0;
}

bool ConnectivityManager::performConnectionDiagnostics() {
    Serial.println("Performing connectivity diagnostics...");
    
    bool bleOK = true;
    bool wifiOK = true;
    
    #if USE_BLE
    if (config.enableBLE) {
        bleOK = (pServer != nullptr && pService != nullptr);
    }
    #endif
    
    #if USE_WIFI
    if (config.enableWiFi) {
        wifiOK = (WiFi.status() == WL_CONNECTED);
    }
    #endif
    
    Serial.printf("BLE Status: %s\n", bleOK ? "OK" : "FAIL");
    Serial.printf("WiFi Status: %s\n", wifiOK ? "OK" : "FAIL");
    
    return bleOK && wifiOK;
}

void ConnectivityManager::shutdown() {
    Serial.println("Shutting down connectivity...");
    
    #if USE_BLE
    if (pServer) {
        pServer->getAdvertising()->stop();
        BLEDevice::deinit();
    }
    #endif
    
    #if USE_WIFI
    if (webServer) {
        webServer->stop();
        delete webServer;
        webServer = nullptr;
    }
    WiFi.disconnect();
    #endif
    
    initialized = false;
    Serial.println("Connectivity shutdown complete");
}