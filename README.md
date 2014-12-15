# Etherbotix Drivers

This ROS package contains python-based ROS drivers for the Etherbotixboard.

## Uploading New Firmware

The upload script will reboot the Etherbotix into the bootloader and then
upload the new firmware. Be sure to disable drivers before doing this:

    roscd etherbotix_python/scripts
    ./upload.py /location/of/firmware.bin
