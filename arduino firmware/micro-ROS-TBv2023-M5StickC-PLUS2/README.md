# Arduino Firmware for micro-ROS Support for Task Board version 2023 (TBv2023)

The firmware is fully compatible with ROS 2 and Task Board version 2023 (TBv2023).

Hold button A while powering on will reset the current Wi-Fi configuration.

You can enter the configuration mode after booting by pressing two two times button A (the M5 button) to go to the configuration screen and then you should hold button B (the button below the display). Afterwards, you can connect to the Wi-Fi provided by the module and open in your browser 192.168.4.1.

Note, that ArduinoOTA and IMU are currently disabled.

There might be some issues with the tzapu's WiFiManager. Sometimes, you need to configure the connection multiple times. Also, if the router is not working or accessible the configuration is being lost. Should be further investigated.