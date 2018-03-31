# Etherbotix Drivers

This ROS package contains python-based ROS drivers for the Etherbotix board.

## Configuring for a Robot

These drivers use a similar configuration strategy to (and updated versions
of controllers from) the arbotix_ros drivers. There is currently no real
documentation, however, see the indigo-devel branch of Maxwell for some
examples.

## Uploading New Firmware

The upload script will reboot the Etherbotix into the bootloader and then
upload the new firmware. Be sure to disable drivers before doing this:

    roscd etherbotix_python/scripts
    ./upload.py /location/of/firmware.bin

## Monitoring Board Status

In addition to the ROS tools, the low level monitor_etherbotix.py is
available.

![Monitor](https://raw.githubusercontent.com/mikeferguson/etherbotix_python/master/docs/monitor.png)
