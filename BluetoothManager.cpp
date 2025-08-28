#include "BluetoothManager.h"

// Static member definitions
BLEServer* BluetoothManager::server = nullptr;
BLECharacteristic* BluetoothManager::dataCharacteristic = nullptr;
BLECharacteristic* BluetoothManager::commandCharacteristic = nullptr;
bool BluetoothManager::deviceConnected = false;
bool BluetoothManager::advertising = false;

// BLE UUIDs
const char* BluetoothManager::SERVICE_UUID = "12345678-1234-1234-1234-123456789abc";
const char* BluetoothManager::DATA_CHAR_UUID = "12345678-1234-1234-1234-123456789abd";
const char* BluetoothManager::COMMAND_CHAR_UUID = "12345678-1234-1234-1234-123456789abe";

// BLE Server Callbacks
class BluetoothManager::BLECallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        BluetoothManager::deviceConnected = true;
        BluetoothManager::advertising = false;
    }
    
    void onDisconnect(BLEServer* pServer) override {
        BluetoothManager::deviceConnected = false;
        // Restart advertising after disconnect
        BluetoothManager::startAdvertising();
    }
};

bool BluetoothManager::initialize(const char* deviceName) {
    // Initialize BLE
    BLEDevice::init(deviceName);
    
    // Create BLE Server
    server = BLEDevice::createServer();
    if (!server) {
        return false;
    }
    
    server->setCallbacks(new BLECallbacks());
    
    // Setup service and characteristics
    setupService();
    setupCharacteristics();
    
    // Start advertising
    startAdvertising();
    
    return true;
}

void BluetoothManager::setupService() {
    // Create BLE Service
    BLEService* service = server->createService(SERVICE_UUID);
    
    // Create characteristics
    dataCharacteristic = service->createCharacteristic(
        DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    commandCharacteristic = service->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    
    // Add descriptors
    dataCharacteristic->addDescriptor(new BLE2902());
    
    // Start service
    service->start();
}

void BluetoothManager::setupCharacteristics() {
    // Set initial values
    dataCharacteristic->setValue("VO2Smart Ready");
}

void BluetoothManager::startAdvertising() {
    if (!advertising && server) {
        BLEAdvertising* advertising_ptr = BLEDevice::getAdvertising();
        advertising_ptr->addServiceUUID(SERVICE_UUID);
        advertising_ptr->setScanResponse(true);
        advertising_ptr->setMinPreferred(0x06);
        advertising_ptr->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
        advertising = true;
    }
}

void BluetoothManager::stopAdvertising() {
    if (advertising) {
        BLEDevice::stopAdvertising();
        advertising = false;
    }
}

bool BluetoothManager::sendData(const String& jsonData) {
    if (deviceConnected && dataCharacteristic) {
        dataCharacteristic->setValue(jsonData.c_str());
        dataCharacteristic->notify();
        return true;
    }
    return false;
}

bool BluetoothManager::isConnected() {
    return deviceConnected;
}

bool BluetoothManager::isAdvertising() {
    return advertising;
}

void BluetoothManager::handleEvents() {
    // Handle any BLE events if needed
    // Currently handled by callbacks
}

void BluetoothManager::disconnect() {
    if (deviceConnected && server) {
        server->disconnect(server->getConnId());
    }
}

String BluetoothManager::getStatusString() {
    if (deviceConnected) {
        return "BLE Connected";
    } else if (advertising) {
        return "BLE Advertising";
    } else {
        return "BLE Disabled";
    }
}