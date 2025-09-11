#pragma once
#include <WiFi.h>
#include <WebServer.h>

/**
 * WiFi Manager - WiFi connectivity and web server management
 */
class WiFiManager {
public:
    enum ConnectionStatus {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        FAILED
    };

private:
    static WebServer server;
    static ConnectionStatus status;
    static String ssid;
    static String password;
    static bool serverStarted;
    static unsigned long lastConnectionAttempt;
    static const unsigned long CONNECTION_TIMEOUT = 30000; // 30 seconds

public:
    /**
     * Initialize WiFi with credentials
     * @param ssid Network name
     * @param password Network password
     * @return true if initialization successful
     */
    static bool initialize(const char* ssid, const char* password);

    /**
     * Start WiFi connection
     * @return true if connection started
     */
    static bool connect();

    /**
     * Disconnect from WiFi
     */
    static void disconnect();

    /**
     * Handle WiFi events and web server
     */
    static void handleEvents();

    /**
     * Get connection status
     * @return Current connection status
     */
    static ConnectionStatus getStatus();

    /**
     * Check if connected
     * @return true if connected
     */
    static bool isConnected();

    /**
     * Get IP address
     * @return IP address string
     */
    static String getIPAddress();

    /**
     * Send JSON data to connected clients
     * @param jsonData JSON string to send
     * @return true if sent successfully
     */
    static bool sendData(const String& jsonData);

private:
    static void setupWebServer();
    static void handleRoot();
    static void handleData();
    static void handleNotFound();
    static void updateStatus();
};