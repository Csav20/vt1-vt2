#include "WiFiManager.h"

// Static member definitions
WebServer WiFiManager::server(80);
WiFiManager::ConnectionStatus WiFiManager::status = DISCONNECTED;
String WiFiManager::ssid = "";
String WiFiManager::password = "";
bool WiFiManager::serverStarted = false;
unsigned long WiFiManager::lastConnectionAttempt = 0;

bool WiFiManager::initialize(const char* networkSSID, const char* networkPassword) {
    ssid = String(networkSSID);
    password = String(networkPassword);
    
    WiFi.mode(WIFI_STA);
    return true;
}

bool WiFiManager::connect() {
    if (status == CONNECTING || status == CONNECTED) {
        return true; // Already connecting or connected
    }
    
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid.c_str(), password.c_str());
    status = CONNECTING;
    lastConnectionAttempt = millis();
    
    return true;
}

void WiFiManager::disconnect() {
    WiFi.disconnect();
    status = DISCONNECTED;
    serverStarted = false;
}

void WiFiManager::handleEvents() {
    updateStatus();
    
    if (status == CONNECTED && serverStarted) {
        server.handleClient();
    }
}

WiFiManager::ConnectionStatus WiFiManager::getStatus() {
    return status;
}

bool WiFiManager::isConnected() {
    return status == CONNECTED;
}

String WiFiManager::getIPAddress() {
    if (status == CONNECTED) {
        return WiFi.localIP().toString();
    }
    return "0.0.0.0";
}

bool WiFiManager::sendData(const String& jsonData) {
    if (status == CONNECTED && serverStarted) {
        // This would typically send to active WebSocket connections
        // For now, we'll just store the data for the next HTTP request
        return true;
    }
    return false;
}

void WiFiManager::setupWebServer() {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.onNotFound(handleNotFound);
    
    server.begin();
    serverStarted = true;
    Serial.println("Web server started");
    Serial.print("Visit: http://");
    Serial.println(WiFi.localIP());
}

void WiFiManager::handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>VO2Smart</title></head><body>";
    html += "<h1>VO2Smart System</h1>";
    html += "<p>Status: Connected</p>";
    html += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
    html += "<p><a href='/data'>View Real-time Data</a></p>";
    html += "<script>";
    html += "setInterval(function(){";
    html += "fetch('/data').then(r=>r.json()).then(d=>{";
    html += "console.log('Data:', d);";
    html += "});";
    html += "}, 1000);";
    html += "</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void WiFiManager::handleData() {
    // Return sample data - in real implementation this would come from sensor data
    String json = "{";
    json += "\"timestamp\":" + String(millis()) + ",";
    json += "\"vo2\":25.5,";
    json += "\"vco2\":23.1,";
    json += "\"rer\":0.91,";
    json += "\"o2\":20.1,";
    json += "\"co2\":450,";
    json += "\"temperature\":25.2,";
    json += "\"humidity\":52";
    json += "}";
    
    server.send(200, "application/json", json);
}

void WiFiManager::handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

void WiFiManager::updateStatus() {
    WiFiStatus_t wifiStatus = WiFi.status();
    
    switch (wifiStatus) {
        case WL_CONNECTED:
            if (status != CONNECTED) {
                status = CONNECTED;
                Serial.println("WiFi connected!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                
                if (!serverStarted) {
                    setupWebServer();
                }
            }
            break;
            
        case WL_NO_SSID_AVAIL:
        case WL_CONNECT_FAILED:
        case WL_CONNECTION_LOST:
            status = FAILED;
            break;
            
        case WL_DISCONNECTED:
            if (status == CONNECTING) {
                // Check for timeout
                if (millis() - lastConnectionAttempt > CONNECTION_TIMEOUT) {
                    status = FAILED;
                    Serial.println("WiFi connection timeout");
                }
            } else if (status == CONNECTED) {
                status = DISCONNECTED;
                serverStarted = false;
                Serial.println("WiFi disconnected");
            }
            break;
            
        default:
            // Still connecting
            break;
    }
}