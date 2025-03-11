#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <M5StickCPlus2.h>

char ros_host[64] = "192.168.1.2";
char ros_port[16] = "8888";
WiFiManager wifiManager;
WiFiManagerParameter ros_host_parameter("ros_host", "ROS Host", ros_host, 40);
WiFiManagerParameter ros_port_parameter("ros_port", "ROS Port", ros_port, 6);
#define CONFIG_FILE "/config.json"
int WIFI_STATUS = -1;
bool portalRunning = false;

void resetSettings() {
  StickCP2.Speaker.tone(8000, 1000);
  wifiManager.resetSettings();
  delay(3000);
  ESP.restart();
  delay(5000);
}

void saveParamsCallback() {
  StaticJsonDocument<512> doc;
  doc["ros_host"] = ros_host_parameter.getValue();
  doc["ros_port"] = ros_port_parameter.getValue();
  File file = SPIFFS.open(CONFIG_FILE, "w");
  serializeJson(doc, file);
  file.close();
}

void setup() {
  // put your setup code here, to run once:
  WiFi.mode(WIFI_STA);
  auto cfg = M5.config();
  StickCP2.begin(cfg);
  Serial.begin(115200);
  if (StickCP2.BtnA.isPressed()) {
    resetSettings();
  }
  delay(1000);
  pinMode(19, OUTPUT);  //GPIO19 for M5StickCPlus2 the builtin LED
  StickCP2.Display.setRotation(3);
  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.setTextColor(WHITE, BLACK);
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" Booting...\n\n Please, wait!");

  SPIFFS.begin(true);
  File file = SPIFFS.open(CONFIG_FILE, "r");
  if (file) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (!error) {
      strcpy(ros_host, doc["ros_host"]);
      strcpy(ros_port, doc["ros_port"]);
      ros_host_parameter.setValue(ros_host, 40);
      ros_port_parameter.setValue(ros_port, 6);
    }
  }
  wifiManager.addParameter(&ros_host_parameter);
  wifiManager.addParameter(&ros_port_parameter);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setSaveParamsCallback(saveParamsCallback);
  delay(3000);
  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" Connecting to\n   Wi-Fi...\n\n Please, wait!");
  if (wifiManager.autoConnect()) {
    Serial.printf("ROS: %s:%s\n", ros_host, ros_port);
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setCursor(0, 2);
    StickCP2.Display.setTextSize(2);
    StickCP2.Display.printf(" Connecting to\n   micro-ROS...\n\n Please, wait!");
  } else {
    wifiManager.stopConfigPortal();
  }
  StickCP2.Display.fillScreen(BLACK);
  screen_idle();
}

void screen_idle() {
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" SSID: %s \n Wi-Fi: %s\n\n ROS: %s:%d\n\n A -> reboot\n B -> config", WiFi.SSID().c_str(), WiFi.status() == WL_CONNECTED ? "OK" : "--", ros_host, atol(ros_port));
}

void screen_config() {
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" CONFIG MODE\n\n Connect to:\n %s\n Go to %s\n A -> reboot\n B -> close\n Hold A -> reset", wifiManager.getConfigPortalSSID().c_str(), WiFi.softAPIP().toString().c_str());
}

void loop() {
  if (portalRunning) {
    wifiManager.process();
  }

  StickCP2.update();
  if (portalRunning) {
    if (StickCP2.BtnA.wasHold()) {
      resetSettings();
    } else if (StickCP2.BtnA.isPressed()) {
      ESP.restart();
    } else if (StickCP2.BtnB.isPressed()) {
      wifiManager.stopConfigPortal();
      portalRunning = false;
      StickCP2.Display.fillScreen(BLACK);
      screen_idle();
      return;
    }
  }
  if (!portalRunning) {
    if (StickCP2.BtnA.isPressed()) {
      ESP.restart();
    } else if (StickCP2.BtnB.isPressed()) {
      wifiManager.startConfigPortal();
      portalRunning = true;
      StickCP2.Display.fillScreen(BLACK);
      screen_config();
      return;
    }
    if (WiFi.status() != WIFI_STATUS) {
      WIFI_STATUS = WiFi.status();
      screen_idle();
    }
  }
}
