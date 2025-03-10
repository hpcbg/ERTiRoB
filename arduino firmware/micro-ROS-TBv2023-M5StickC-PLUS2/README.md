# Arduino Firmware for micro-ROS Support for Task Board version 2023 (TBv2023)

The firmware is fully compatible with ROS 2 and Task Board version 2023 (TBv2023).

Hold button A while powering on to force into config mode.

Note, that ArduinoOTA and IMU are currently disabled.

There are some issues with the tzapu's WiFiManager. Sometimes, you need to configure the connection multiple times. Also, if the router is not working or accessible the configuration is being lost. Should be further investigated.