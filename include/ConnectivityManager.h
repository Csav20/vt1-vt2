#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/**
 * @brief Enhanced connectivity management with automatic reconnection and robust error handling
 * 
 * Features:
 * - Automatic BLE reconnection with exponential backoff
 * - Robust WiFi management with multiple AP support
 * - Comprehensive error handling and recovery
 * - Optimized data transmission protocols
 * - Real-time connection monitoring
 */
class ConnectivityManager {
public:
    struct Config {
        bool enableBLE;
        bool enableWiFi;
        bool dualMode;              // Allow simultaneous BLE and WiFi
        const char* deviceName;
        const char* wifiSSID;
        const char* wifiPassword;
        const char* fallbackSSID;   // Backup WiFi network
        const char* fallbackPassword;
        uint16_t webServerPort;
        uint32_t bleAdvertisingInterval; // ms
        uint32_t reconnectInterval;      // ms
        uint8_t maxReconnectAttempts;
        bool enableEncryption;
    };

    enum ConnectionStatus {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        RECONNECTING,
        FAILED,
        DISABLED
    };

    struct ConnectionState {
        ConnectionStatus bleStatus;
        ConnectionStatus wifiStatus;
        uint32_t bleConnectedTime;
        uint32_t wifiConnectedTime;
        uint32_t lastBLEAttempt;
        uint32_t lastWiFiAttempt;
        uint8_t bleReconnectCount;
        uint8_t wifiReconnectCount;
        int8_t wifiSignalStrength;
        uint8_t connectedClients;
    };

    // Core connectivity functions
    static void begin(const Config& config);
    static void update();
    static void shutdown();
    
    // Connection management
    static bool isBLEConnected();
    static bool isWiFiConnected();
    static bool isAnyConnectionActive();
    static ConnectionState getConnectionState();
    
    // Data transmission
    static bool sendData(const String& jsonData);
    static bool sendDataBLE(const uint8_t* data, size_t length);
    static bool sendDataWiFi(const String& data);
    static void broadcastData(const String& data);
    
    // Connection control
    static void enableBLE(bool enable);
    static void enableWiFi(bool enable);
    static void forceReconnectBLE();
    static void forceReconnectWiFi();
    static void resetConnections();
    
    // Error handling and recovery
    static const char* getLastError();
    static uint32_t getErrorCount();
    static void clearErrors();
    static bool performConnectionDiagnostics();
    
    // Advanced features
    static void setQualityOfService(uint8_t qos);
    static void enableDataCompression(bool enable);
    static void setTransmissionRate(uint16_t packetsPerSecond);
    static uint32_t getDataThroughput();
    
    // Client management (for web server)
    static void handleClients();
    static uint8_t getConnectedClientCount();
    static void setClientTimeout(uint32_t timeoutMs);

private:
    // Configuration and state
    static Config config;
    static ConnectionState state;
    static bool initialized;
    
    // BLE components
    static BLEServer* pServer;
    static BLEService* pService;
    static BLECharacteristic* pDataCharacteristic;
    static BLECharacteristic* pControlCharacteristic;
    static BLECharacteristic* pStatusCharacteristic;
    static BLEAdvertising* pAdvertising;
    
    // WiFi components
    static WebServer* webServer;
    static IPAddress localIP;
    static IPAddress gateway;
    static IPAddress subnet;
    
    // Error tracking
    static char lastError[128];
    static uint32_t errorCount;
    static uint32_t lastErrorTime;
    
    // Reconnection management
    static uint32_t nextBLEReconnect;
    static uint32_t nextWiFiReconnect;
    static uint32_t bleBackoffDelay;
    static uint32_t wifiBackoffDelay;
    
    // Data transmission optimization
    static bool compressionEnabled;
    static uint16_t transmissionRate;
    static uint32_t lastTransmission;
    static uint32_t bytesTransmitted;
    static uint32_t throughputCalculationTime;
    
    // BLE implementation
    static void setupBLE();
    static void startBLEAdvertising();
    static void stopBLEAdvertising();
    static bool reconnectBLE();
    static void handleBLEDisconnection();
    
    // WiFi implementation
    static void setupWiFi();
    static bool connectToWiFi(const char* ssid, const char* password);
    static bool reconnectWiFi();
    static void handleWiFiDisconnection();
    static void setupWebServer();
    static void handleWebRequests();
    
    // Error handling
    static void logError(const char* source, const char* message);
    static void updateConnectionStatus();
    static bool shouldAttemptReconnection(ConnectionStatus status, uint32_t lastAttempt, uint32_t interval);
    static void calculateBackoffDelay(uint32_t& backoffDelay, uint8_t attemptCount);
    
    // Data processing
    static String compressData(const String& data);
    static String decompressData(const String& data);
    static bool validateDataIntegrity(const String& data);
    static void updateThroughputStats(size_t bytes);
    
    // Web server handlers
    static void handleRoot();
    static void handleData();
    static void handleStatus();
    static void handleConfig();
    static void handleNotFound();
    static void handleCORS();
    
    // BLE callbacks
    class ServerCallbacks;
    class CharacteristicCallbacks;
    
    // Constants
    static const char* SERVICE_UUID;
    static const char* DATA_CHAR_UUID;
    static const char* CONTROL_CHAR_UUID;
    static const char* STATUS_CHAR_UUID;
    static const uint32_t BLE_MTU_SIZE = 512;
    static const uint32_t MAX_BACKOFF_DELAY = 60000;  // 1 minute
    static const uint32_t MIN_BACKOFF_DELAY = 1000;   // 1 second
    static const uint32_t CONNECTION_TIMEOUT = 15000;  // 15 seconds
    static const uint32_t THROUGHPUT_CALCULATION_INTERVAL = 10000; // 10 seconds
};