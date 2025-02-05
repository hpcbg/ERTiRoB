#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#define CONFIG_FILE "/config.json"

class NetworkManager {
private:
  String wifi_ssid;
  String wifi_password;
  String ros_host;
  int ros_port;

  bool saveConfig();
  bool loadConfig();

public:
  NetworkManager()
    : wifi_ssid(""), wifi_password(""), ros_host(""), ros_port(0) {
  }

  static NetworkManager* instance;
  bool shouldSaveConfig = false;

  void setup();
  void resetConfig();

  // Getter and Setter methods
  String getWifiSSID();
  void setWifiSSID(const String& ssid);

  String getWifiPassword();
  void setWifiPassword(const String& password);

  String getRosHost();
  void setRosHost(const String& host);

  int getRosPort();
  void setRosPort(int port);
};

#endif  // NETWORK_MANAGER_H
