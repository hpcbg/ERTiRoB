#include "network_setup.h"  // Only include the header once here
#include <micro_ros_arduino.h>
#include <ESPmDNS.h>  // For ESP32/ESP8266

#define CP() Serial.printf("%s:%d\n", __FUNCTION__, __LINE__)

void NetworkManager::setupOTA() {
  if (!MDNS.begin("esp")) {
    Serial.println("Error setting up MDNS");
    return;
  }

  ArduinoOTA.onStart([]() {
    String updateType = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("OTA Update Start: " + updateType);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete! Restarting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}


bool NetworkManager::saveConfig() {
  StaticJsonDocument<256> doc;
  doc["wifi_ssid"] = wifi_ssid;
  doc["wifi_password"] = wifi_password;
  doc["ros_host"] = ros_host;
  doc["ros_port"] = ros_port;

  File file = SPIFFS.open(CONFIG_FILE, "w");
  if (!file) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  serializeJson(doc, file);
  file.close();
  return true;
}

bool NetworkManager::loadConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return false;
  }

  File file = SPIFFS.open(CONFIG_FILE, "r");
  if (!file) {
    Serial.println("Failed to open config file");
    return false;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }

  wifi_ssid = doc["wifi_ssid"].as<String>();
  wifi_password = doc["wifi_password"].as<String>();
  ros_host = doc["ros_host"].as<String>();
  ros_port = doc["ros_port"];

  Serial.printf("ROS: %s:%d\n", ros_host, ros_port);

  return true;
}

void NetworkManager::resetConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
  }

  bool retval = SPIFFS.remove(CONFIG_FILE);
  Serial.printf("resetConfig: %d\n", retval);
}


void NetworkManager::setup() {
  bool loaded = loadConfig();

  addParameter(&custom_ros_host);
  addParameter(&custom_ros_port);

  setSaveConfigCallback([this]() {
    Serial.println("Saving configuration...");

    wifi_ssid = WiFi.SSID();     // Get current WiFi SSID
    wifi_password = WiFi.psk();  // Get current WiFi passwor
    ros_host = custom_ros_host.getValue();
    ros_port = String(custom_ros_port.getValue()).toInt();
    saveConfig();
  });

  setPreOtaUpdateCallback([]() {
    Serial.println("OTA update starting, preparing system...");
  });

  if (!loaded) {
    startConfigPortal();
    ros_host = custom_ros_host.getValue();
    ros_port = String(custom_ros_port.getValue()).toInt();

    saveConfig();
  }

  Serial.println("Configuring Micro-ROS WiFi transport...");
  set_microros_wifi_transports(
    const_cast<char*>(wifi_ssid.c_str()),
    const_cast<char*>(wifi_password.c_str()),
    const_cast<char*>(ros_host.c_str()),
    ros_port);

  setupOTA();
}

String NetworkManager::getWifiSSID() const {
  return wifi_ssid;
}

String NetworkManager::getWifiPassword() const {
  return wifi_password;
}

String NetworkManager::getRosHost() const {
  return ros_host;
}

int NetworkManager::getRosPort() {
  return ros_port;
}
