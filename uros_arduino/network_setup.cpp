#include "network_setup.h"  // Only include the header once here
#include <micro_ros_arduino.h>

static bool shouldSaveConfig = false;

bool NetworkManager::saveConfig() {
    Serial.println("saveConfig enter");

    StaticJsonDocument<512> jsonConfig;
    jsonConfig["wifi_ssid"] = wifi_ssid;
    jsonConfig["wifi_password"] = wifi_password;
    jsonConfig["ros_host"] = ros_host;
    jsonConfig["ros_port"] = String(ros_port);

    File configFile = SPIFFS.open(CONFIG_FILE, "w");
    if (!configFile) {
        Serial.println("Failed to open config file for writing");
        return false;
    }

    serializeJson(jsonConfig, configFile);
    configFile.close();

    Serial.println("saveConfig exit");
    return true;
}

bool NetworkManager::loadConfig() {    
    if (!SPIFFS.exists(CONFIG_FILE)) {
        Serial.println("Config file does not exist");
    
        return false;
    }

    File configFile = SPIFFS.open(CONFIG_FILE, "r");
    if (!configFile) {
        Serial.println("Failed to open config file for reading");
        return false;
    }

    StaticJsonDocument<512> jsonConfig;
    DeserializationError error = deserializeJson(jsonConfig, configFile);
    if (error) {
        Serial.println("Failed to parse config file");
        return false;
    }

    wifi_ssid = jsonConfig["wifi_ssid"].as<String>();
    wifi_password = jsonConfig["wifi_password"].as<String>();
    ros_host = jsonConfig["ros_host"].as<String>();
    ros_port = jsonConfig["ros_port"].as<int>();

    configFile.close();

    return true;
}

void NetworkManager::resetConfig() {
    SPIFFS.remove(CONFIG_FILE);

    Serial.println("Reset network config");
    delay(3000);
    ESP.restart();
}

void saveConfigCallback() {   
      Serial.println("Configuration changed, marking for save...");
      shouldSaveConfig = true;   
}

void NetworkManager::setup() {
    WiFiManager wifiManager;
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    loadConfig();

    // Start WiFiManager configuration portal if needed
    bool shouldStartConfigPortal = (wifi_ssid.length() == 0 
        || wifi_password.length() == 0
        || ros_host.length() == 0
        || ros_port == 0);

    if (shouldStartConfigPortal) {
      // Set the save configuration callback
      wifiManager.setSaveConfigCallback(saveConfigCallback);

      // WiFiManager parameters for ROS settings
      WiFiManagerParameter custom_ros_host("ros_host", "ROS host", "192.168.88.10", 40);
      wifiManager.addParameter(&custom_ros_host);
      ros_host = custom_ros_host.getValue();

      WiFiManagerParameter custom_ros_port("ros_port", "ROS port", "8888", 6);
      wifiManager.addParameter(&custom_ros_port);
      ros_port = atoi(custom_ros_port.getValue());

      Serial.println("Starting configuration portal...");
      if (!wifiManager.startConfigPortal("ESP32-Provisioning")) {
          Serial.println("Failed to connect and hit timeout");
          delay(3000);
          ESP.restart();
      }

      wifi_ssid = WiFi.SSID();
      wifi_password = WiFi.psk();
    } else {
      Serial.println("Using loaded config...");
      WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());     
    }

    delay(2000);  // Wait 2 seconds to ensure WiFi is fully connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi is not connected.");
    } else {
        Serial.println("WiFi is connected.");
        if(shouldStartConfigPortal) {
          Serial.println("Saving config...");
          saveConfig();
        }
    }
}

// Getter and Setter methods
String NetworkManager::getWifiSSID() {
    return wifi_ssid;
}

void NetworkManager::setWifiSSID(const String& ssid) {
    wifi_ssid = ssid;
    saveConfig();
}

String NetworkManager::getWifiPassword() {
    return wifi_password;
}

void NetworkManager::setWifiPassword(const String& password) {
    wifi_password = password;
    saveConfig();
}

String NetworkManager::getRosHost() {
    return ros_host;
}

void NetworkManager::setRosHost(const String& host) {
    ros_host = host;
    saveConfig();
}

int NetworkManager::getRosPort() {
    return ros_port;
}

void NetworkManager::setRosPort(int port) {
    ros_port = port;
    saveConfig();
}