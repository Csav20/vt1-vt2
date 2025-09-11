#pragma once
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/**
 * Bluetooth Manager - BLE only implementation
 * Removes BluetoothSerial conflict and uses only BLE
 */
class BluetoothManager {
public:
    struct BLECallbacks;
    
private:
    static BLEServer* server;
    static BLECharacteristic* dataCharacteristic;
    static BLECharacteristic* commandCharacteristic;
    static bool deviceConnected;
    static bool advertising;
    
    // BLE UUIDs
    static const char* SERVICE_UUID;
    static const char* DATA_CHAR_UUID;
    static const char* COMMAND_CHAR_UUID;
    
public:
    /**
     * Initialize BLE
     * @param deviceName Name for BLE device
     * @return true if successful
     */
    static bool initialize(const char* deviceName = "VO2Smart");
    
    /**
     * Start advertising
     */
    static void startAdvertising();
    
    /**
     * Stop advertising
     */
    static void stopAdvertising();
    
    /**
     * Send data via BLE
     * @param jsonData JSON string to send
     * @return true if sent successfully
     */
    static bool sendData(const String& jsonData);
    
    /**
     * Check if device is connected
     * @return true if connected
     */
    static bool isConnected();
    
    /**
     * Check if advertising
     * @return true if advertising
     */
    static bool isAdvertising();
    
    /**
     * Handle BLE events (call in main loop)
     */
    static void handleEvents();
    
    /**
     * Disconnect current client
     */
    static void disconnect();
    
    /**
     * Get connection status string
     * @return Status string
     */
    static String getStatusString();
    
private:
    static void setupService();
    static void setupCharacteristics();
};