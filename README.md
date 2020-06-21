# Etherbotix Drivers

This ROS2 package contains ROS drivers for the Etherbotix board.

## Uploading New Firmware

The upload script will reboot the Etherbotix into the bootloader and then
upload the new firmware. Be sure to disable drivers before doing this:

    ros2 run etherbotix upload /location/of/firmware.bin

## Monitoring Board Status

In addition to the ROS tools, the low level _monitor_ is available.

![Monitor](https://raw.githubusercontent.com/mikeferguson/etherbotix_python/ros2/docs/monitor.png)

The digital IO section uses the following notation:
 * A = analog input
 * I/O = digital input or output respectively
 * U = pin is in use by user io (SPI, USART, etc)
 * H/L = digital status of pin, high or low.
