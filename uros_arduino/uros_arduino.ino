#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <M5StickCPlus2.h>

#include "network_setup.h"
NetworkManager nm;


rcl_publisher_t battery_publisher;
rcl_publisher_t buttonA_publisher;
rcl_publisher_t buttonB_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t tf_publisher;
std_msgs__msg__Int32 battery_msg;
std_msgs__msg__Int32 buttonA_msg;
std_msgs__msg__Int32 buttonB_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { Serial.println("RCCHECK: " + temp_rc); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { Serial.println("RCSOFTCHECK: " + temp_rc); } \
  }

void ros_subscribe() {
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher 1
  RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "battery/voltage"));

  // create publisher 2
  RCCHECK(rclc_publisher_init_default(
    &buttonA_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "button/A_status"));

  // create publisher 3
  RCCHECK(rclc_publisher_init_default(
    &buttonB_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "button/B_status"));

  // create publisher 4
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));


  // Initialize IMU message
  imu_msg.orientation_covariance[0] = -1;  // Mark orientation as unavailable
  imu_msg.header.frame_id.data = (char *)"imu_frame";
  imu_msg.header.frame_id.size = strlen("imu_frame");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
}

void ros_publish() {
  static int last_battery, last_buttonA, last_buttonB;
  static float last_gyro_x, last_gyro_y, last_gyro_z;
  static float last_accel_x, last_accel_y, last_accel_z;

  auto imu_update = StickCP2.Imu.update();
  int vol = StickCP2.Power.getBatteryVoltage();

  if (vol != last_battery) {
    battery_msg.data = vol;
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    last_battery = vol;
  }

  int stateA = StickCP2.BtnA.getState();
  if (stateA != last_buttonA) {
    buttonA_msg.data = stateA;
    RCSOFTCHECK(rcl_publish(&buttonA_publisher, &buttonA_msg, NULL));
    last_buttonA = stateA;
  }

  int stateB = StickCP2.BtnB.getState();
  if (stateB != last_buttonB) {
    buttonB_msg.data = stateB;
    RCSOFTCHECK(rcl_publish(&buttonB_publisher, &buttonB_msg, NULL));
    last_buttonB = stateB;
  }

  auto data = StickCP2.Imu.getImuData();

  float accel_thresh = 0.05;  // Example threshold for acceleration
  float gyro_thresh = 5;      // Example threshold for gyroscope

  bool accel_changed = (fabs(data.accel.x - last_accel_x) > accel_thresh || fabs(data.accel.y - last_accel_y) > accel_thresh || fabs(data.accel.z - last_accel_z) > accel_thresh);

  bool gyro_changed = (fabs(data.gyro.x - last_gyro_x) > gyro_thresh || fabs(data.gyro.y - last_gyro_y) > gyro_thresh || fabs(data.gyro.z - last_gyro_z) > gyro_thresh);

  if (accel_changed || gyro_changed) {
    imu_msg.linear_acceleration.x = data.accel.x;
    imu_msg.linear_acceleration.y = data.accel.y;
    imu_msg.linear_acceleration.z = data.accel.z;
    imu_msg.angular_velocity.x = data.gyro.x;
    imu_msg.angular_velocity.y = data.gyro.y;
    imu_msg.angular_velocity.z = data.gyro.z;
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    last_accel_x = data.accel.x;
    last_accel_y = data.accel.y;
    last_accel_z = data.accel.z;
    last_gyro_x = data.gyro.x;
    last_gyro_y = data.gyro.y;
    last_gyro_z = data.gyro.z;
  }
}

void handlePreOtaUpdateCallback() {
  Update.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("CUSTOM Progress: %u%%\r", (progress / (total / 100)));
  });
}

void loop() {
  ros_publish();

  ArduinoOTA.handle();
}

void setup() {
  Serial.begin(115200);

  auto cfg = M5.config();
  StickCP2.begin(cfg);
  StickCP2.Display.setRotation(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  StickCP2.Display.setTextDatum(middle_center);
  StickCP2.Display.setTextFont(&fonts::Orbitron_Light_24);
  StickCP2.Display.setTextSize(1);

  StickCP2.update();
  StickCP2.Display.clear();

  StickCP2.Display.setCursor(10, 10);
  StickCP2.Display.printf("Starting .......\n");

  if (StickCP2.BtnA.isPressed()) {
    StickCP2.Speaker.tone(8000, 20);
    nm.resetConfig();

    Serial.flush();
    delay(300);
    ESP.restart();
  }

  nm.setup();

  StickCP2.Display.clear();
  M5.Display.println("Connected!");

  Serial.println("Connected.");

  Serial.println("[WiFi SSID]: " + nm.getWifiSSID());
  Serial.println("[WiFi IP]: " + nm.getLocalIP());
  Serial.println("[ROS Host]: " + nm.getRosHost());
  Serial.println("[ROS Port]: " + String(nm.getRosPort()));

  ros_subscribe();
}
