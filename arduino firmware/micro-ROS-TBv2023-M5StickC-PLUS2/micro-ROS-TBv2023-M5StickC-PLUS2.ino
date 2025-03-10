#include <stdio.h>

#include <M5StickCPlus2.h>
#include "porthub.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcutils/allocator.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include "network_setup.h"

//-----------------------------------------------------------------------------------------
// Global variables
int verbose = 0;  // set to 1 to enable serial output, default 0

String MAC = "";
unsigned long now = 0;
unsigned long last = 0;
unsigned long scan_time = -1;
int screen_selector = 0;  // int to navigate multiple screens

NetworkManager nm;
// End of global variables
//-----------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------
// Sensors
PortHub porthub;
uint8_t HUB_ADDR[6] = { HUB1_ADDR, HUB2_ADDR, HUB3_ADDR, HUB4_ADDR, HUB5_ADDR, HUB6_ADDR };

int buttonA = -1, buttonB = -1, voltage = -1;
int buttonPushState = -1;  // blue button (0 - pressed, 1 - released)
int stopBtnState = -1;     // red button (0 - pressed, 1 - released)
int faderValue = -1;       // fader (slider) sensor (0 - up, >4000 - down)
int angleValue = -1;       // angle (door) sensor (2400 - open, >4000 - closed)
int probeStartState = -1;  // test probe in terminal block (0 - touching, 1 - not there)
int probeGoalState = -1;   // test probe in holder (0 - touching, 1 - not there)
int OP180_1_State = -1;    // left cable wrap post (0 - no cable, 1 - cable wrapped)
int OP180_2_State = -1;    // right cale wrap post (0 - no cable, 1 - cable wrapped)

void update_inputs() {
  voltage = StickCP2.Power.getBatteryVoltage();

  // Read from PbHub Module
  porthub.hub_d_wire_value_A(HUB_ADDR[3], 1);                 // write value high, this is the solution that fixed the floating values from the new STM vs MEGA chips on the PbHub
  porthub.hub_d_wire_value_B(HUB_ADDR[3], 1);                 // write value high, this is the solution that fixed the floating values from the new STM vs MEGA chips on the PbHub
  buttonPushState = porthub.hub_d_read_value_A(HUB_ADDR[0]);  // blue button
  stopBtnState = porthub.hub_d_read_value_B(HUB_ADDR[0]);     // red button
  faderValue = porthub.hub_a_read_value(HUB_ADDR[5]);         // fader
  angleValue = porthub.hub_a_read_value(HUB_ADDR[4]);         // angle
  probeStartState = porthub.hub_d_read_value_A(HUB_ADDR[3]);  // probe in terminal block
  probeGoalState = porthub.hub_d_read_value_B(HUB_ADDR[3]);   // probe in holder
  OP180_1_State = porthub.hub_d_read_value_A(HUB_ADDR[1]);    // left post
  OP180_2_State = porthub.hub_d_read_value_A(HUB_ADDR[2]);    // right post
}
// End of sensors
//-----------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------
// ROS 2
rcl_publisher_t heartbeat_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t buttonPushState_publisher;
rcl_publisher_t stopBtnState_publisher;
rcl_publisher_t faderValue_publisher;
rcl_publisher_t angleValue_publisher;
rcl_publisher_t probeStartState_publisher;
rcl_publisher_t probeGoalState_publisher;
rcl_publisher_t OP180_1_State_publisher;
rcl_publisher_t OP180_2_State_publisher;

std_msgs__msg__String heartbeat_msg;
std_msgs__msg__Int32 battery_msg;
std_msgs__msg__Int32 buttonPushState_msg;
std_msgs__msg__Int32 stopBtnState_msg;
std_msgs__msg__Int32 faderValue_msg;
std_msgs__msg__Int32 angleValue_msg;
std_msgs__msg__Int32 probeStartState_msg;
std_msgs__msg__Int32 probeGoalState_msg;
std_msgs__msg__Int32 OP180_1_State_msg;
std_msgs__msg__Int32 OP180_2_State_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool ROS_OK = false;

#define HEARTBEAT_TIMEOUT 10000
unsigned long last_heartbeat_time = 0;

#define RCCHECK(fn) \
  { \
    if (!ROS_OK) return; \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.println("ROS Error!"); \
      ROS_OK = false; \
    } \
  }

void ros_subscribe() {
  if (WiFi.status() != WL_CONNECTED) {
    ROS_OK = false;
    return;
  }
  ROS_OK = true;

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  char node_name[24];
  sprintf(node_name, "task_board_%s\0", MAC.c_str());

  RCCHECK(rclc_node_init_default(&node, "task_board", node_name, &support));

  RCCHECK(rclc_publisher_init_default(
    &heartbeat_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/task_board_heartbeat"));

  RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "voltage"));

  RCCHECK(rclc_publisher_init_default(
    &buttonPushState_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "buttonPushState"));

  RCCHECK(rclc_publisher_init_default(
    &stopBtnState_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stopBtnState"));

  RCCHECK(rclc_publisher_init_default(
    &faderValue_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "faderValue"));

  RCCHECK(rclc_publisher_init_default(
    &angleValue_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "angleValue"));

  RCCHECK(rclc_publisher_init_default(
    &probeStartState_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "probeStartState"));

  RCCHECK(rclc_publisher_init_default(
    &probeGoalState_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "probeGoalState"));

  RCCHECK(rclc_publisher_init_default(
    &OP180_1_State_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "OP180_1_State"));

  RCCHECK(rclc_publisher_init_default(
    &OP180_2_State_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "OP180_2_State"));

  // Initialize Heartbeat message
  heartbeat_msg.data.data = (char *)malloc(12 * sizeof(char));
  heartbeat_msg.data.size = 13;
  heartbeat_msg.data.capacity = 13;
  sprintf(heartbeat_msg.data.data, "%s", MAC);
  heartbeat_msg.data.data[12] = 0;
}

void ros_publish() {
  battery_msg.data = voltage;
  RCCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));

  buttonPushState_msg.data = buttonPushState;
  RCCHECK(rcl_publish(&buttonPushState_publisher, &buttonPushState_msg, NULL));

  stopBtnState_msg.data = stopBtnState;
  RCCHECK(rcl_publish(&stopBtnState_publisher, &stopBtnState_msg, NULL));

  faderValue_msg.data = faderValue;
  RCCHECK(rcl_publish(&faderValue_publisher, &faderValue_msg, NULL));

  angleValue_msg.data = angleValue;
  RCCHECK(rcl_publish(&angleValue_publisher, &angleValue_msg, NULL));

  probeStartState_msg.data = probeStartState;
  RCCHECK(rcl_publish(&probeStartState_publisher, &probeStartState_msg, NULL));

  probeGoalState_msg.data = probeGoalState;
  RCCHECK(rcl_publish(&probeGoalState_publisher, &probeGoalState_msg, NULL));

  OP180_1_State_msg.data = OP180_1_State;
  RCCHECK(rcl_publish(&OP180_1_State_publisher, &OP180_1_State_msg, NULL));

  OP180_2_State_msg.data = OP180_2_State;
  RCCHECK(rcl_publish(&OP180_2_State_publisher, &OP180_2_State_msg, NULL));

  // Send heartbeat message
  unsigned long time = millis();
  if (time - last_heartbeat_time >= HEARTBEAT_TIMEOUT) {
    last_heartbeat_time = time;
    RCCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
  }
}
// End of ROS 2
//-----------------------------------------------------------------------------------------

void update_screen() {
  // Screen switching logic
  if (screen_selector < 100) {
    if (StickCP2.BtnA.wasReleased()) {
      StickCP2.Display.fillScreen(BLACK);
      ++screen_selector;
      if (screen_selector == 15) {
        screen_selector = 0;
      }
    }
  }

  if (screen_selector == 0) {
    if (StickCP2.BtnB.wasReleased()) {
      if (!ROS_OK) {
        ESP.restart();
        StickCP2.Display.fillScreen(BLACK);
      }
    }
  } else if (screen_selector == 1) {
    if (StickCP2.BtnB.wasReleased()) {
      StickCP2.Display.fillScreen(BLACK);
      screen_selector = 101;
    }
  } else if (screen_selector == 2) {
    if (StickCP2.BtnB.wasHold()) {
      nm.resetConfig();
      delay(300);
      ESP.restart();
    }
  } else if (screen_selector == 101) {
    if (StickCP2.BtnB.wasReleased()) {
      StickCP2.Display.fillScreen(BLACK);
      screen_selector = 1;
    } else {
      if (StickCP2.BtnA.wasReleased()) {
        ESP.restart();
      } else if (StickCP2.BtnA.wasHold()) {
        nm.resetConfig();
        delay(300);
        ESP.restart();
      }
    }
  }

  switch (screen_selector) {
    case 0:
      screen_header();
      if (!ROS_OK) {
        StickCP2.Display.printf(" Press button B to\n   RECONNECT to ROS");
      }
      break;
    case 1:
      screen_header();
      StickCP2.Display.printf(" Press button B to\n   REBOOT or RESET");
      break;
    case 2:
      screen_header();
      StickCP2.Display.printf(" Hold button B to\n   ENTER config mode");
      break;
    case 3:
      screen_header();
      StickCP2.Display.printf(" ROS:\n %s:%s\n", nm.getRosHost(), String(nm.getRosPort()));
      break;
    case 4:
      screen_header();
      StickCP2.Display.printf(" Wi-Fi SSID:\n %s\n", nm.getWifiSSID());
      break;
    case 5:
      screen_header();
      StickCP2.Display.printf(" Wi-Fi IP:\n %s\n", nm.getLocalIP());
      break;
    case 6:
      screen_header();
      StickCP2.Display.printf(" MAC: %s\n", MAC.c_str());
      break;
    case 7:
      screen_header();
      StickCP2.Display.printf(" buttonPushState: %d\n", buttonPushState);
      break;
    case 8:
      screen_header();
      StickCP2.Display.printf(" stopBtnState: %d\n", stopBtnState);
      break;
    case 9:
      screen_header();
      StickCP2.Display.printf(" faderValue: %4d\n", faderValue);
      break;
    case 10:
      screen_header();
      StickCP2.Display.printf(" angleValue: %4d\n", angleValue);
      break;
    case 11:
      screen_header();
      StickCP2.Display.printf(" probeStartState: %d\n", probeStartState);
      break;
    case 12:
      screen_header();
      StickCP2.Display.printf(" probeGoalState: %d\n", probeGoalState);
      break;
    case 13:
      screen_header();
      StickCP2.Display.printf(" OP180_1_State: %d\n", OP180_1_State);
      break;
    case 14:
      screen_header();
      StickCP2.Display.printf(" OP180_2_State: %d\n", OP180_2_State);
      break;
    case 101:
      StickCP2.Display.setCursor(2, 2);
      StickCP2.Display.setTextSize(2);
      StickCP2.Display.printf(" HOLD button A\n   to RESET settings");
      StickCP2.Display.setCursor(2, 52);
      StickCP2.Display.printf(" Press button A\n   to REBOOT\n");
      StickCP2.Display.setCursor(2, 102);
      StickCP2.Display.printf(" Press button B\n   to CANCEL\n");
      break;
  }
}

void screen_header() {
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" Wi-Fi: %s, ROS: %s\n",
                          WiFi.status() == WL_CONNECTED ? "OK" : "--",
                          ROS_OK ? "OK" : "--");
  StickCP2.Display.printf(" Battery: %d%% %0.1fV\n\n",
                          (int)StickCP2.Power.getBatteryLevel(),
                          (float)StickCP2.Power.getBatteryVoltage() / 1000);
  StickCP2.Display.printf(" Press button A to\n   SELECT screen\n\n");
}

void setup() {
  auto cfg = M5.config();
  StickCP2.begin(cfg);
  if (StickCP2.BtnA.isPressed()) {
    StickCP2.Speaker.tone(8000, 1000);
    nm.resetConfig();
    Serial.flush();
    delay(300);
    ESP.restart();
  }
  porthub.begin();
  Serial.begin(115200);
  delay(1000);
  pinMode(19, OUTPUT);  //GPIO19 for M5StickCPlus2 the builtin LED

  // Configure LCD display setup
  StickCP2.Display.setRotation(3);
  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.setTextColor(WHITE, BLACK);
  StickCP2.Display.setCursor(0, 2);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.printf(" Booting.\n\n Please, wait...");

  String mac = WiFi.macAddress();
  for (int i = 0; i < mac.length(); ++i) {
    if (mac[i] != ':')
      MAC = MAC + mac[i];
  }

  nm.setup();

  delay(500);

  if (WiFi.status() == WL_CONNECTED) {
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextColor(WHITE, BLACK);
    StickCP2.Display.setCursor(0, 2);
    StickCP2.Display.setTextSize(2);
    StickCP2.Display.printf(" Initializing\n   ROS node...\n\n Please, wait!");
    ros_subscribe();

    delay(500);
  } else {
    ROS_OK = false;
  }

  StickCP2.Display.fillScreen(BLACK);
}

void loop() {
  StickCP2.update();

  last = now;
  now = millis();
  scan_time = now - last;

  update_inputs();

  update_screen();

  if (ROS_OK) {
    ros_publish();
  }

  if (verbose == 1) {
    Serial.printf("buttonPushState:%d, stopBtnState:%d, faderValue:%d, angleValue:%d, ",
                  buttonPushState, stopBtnState, faderValue, angleValue);
    Serial.printf("probeStartState:%d, probeGoalState:%d, OP180_1_State:%d, OP180_2_State:%d, ",
                  probeStartState, probeGoalState, OP180_1_State, OP180_2_State);
    Serial.printf("scan time: %dms\n", scan_time);
  }
}