#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#define CONFIG_FILE "/config.json"


class NetworkManager : public WiFiManager {
private:
  String wifi_ssid;
  String wifi_password;
  String ros_host;
  int ros_port;
  
  WiFiManagerParameter custom_ros_host;
  WiFiManagerParameter custom_ros_port;

  bool saveConfig();
  bool loadConfig();

public:
    NetworkManager() 
    : custom_ros_host("ros_host", "ROS Host", "192.168.0.1", 40), 
      custom_ros_port("ros_port", "ROS Port", "8888", 6) {}

  void setup();
  void resetConfig();
  
  String getWifiSSID() const;
  String getWifiPassword() const;

  String getRosHost() const;
  int getRosPort();
};

#endif  // NETWORK_MANAGER_H
