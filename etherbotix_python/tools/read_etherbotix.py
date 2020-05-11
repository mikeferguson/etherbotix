#!/usr/bin/env python

# Copyright (c) 2014-2018 Michael Ferguson
# Copyright (c) 2008-2013 Vanadium Labs LLC.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time
from etherbotix_python.ax12 import AX_READ_DATA
from etherbotix_python.etherbotix import Etherbotix


def get_baud_str(baud):
    if baud == 1:
        return "1mbps"
    elif baud == 3:
        return "5kbps"
    elif baud == 16:
        return "115200"
    elif baud == 34:
        return "57600"
    elif baud == 103:
        return "19200"
    elif baud == 207:
        return "9600"
    elif baud == 250:
        return "2250000"
    elif baud == 251:
        return "2500000"
    elif baud == 252:
        return "3000000"
    else:
        return "UNKNOWN"


def main(args=None):
    e = Etherbotix()

    pkt = None
    while pkt is None:
        pkt = e.execute(253, AX_READ_DATA, [0, 128])
        if pkt is None:
            print("Waiting for EtherbotiX response!")
            time.sleep(0.5)
    e.updateFromPacket(pkt)

    print("System Time:    %d" % e.system_time)
    print("Version:        %d" % e.version)
    print("Unique ID:      %s" % e.getUniqueId())
    print("Packets Recv:   %d" % e.packets_recv)
    print("Packets Bad:    %d" % e.packets_bad)
    print("Baud Rate:      %d (%s)" % (e.baud_rate, get_baud_str(e.baud_rate)))
    print("Digital In:     %d" % e.digital_in)
    print("Digital Out:    %d" % e.digital_out)
    print("Digital Dir:    %d" % e.digital_dir)
    print("Alarm LED:      %d" % e.led)
    print("System Voltage: %f V" % e.system_voltage)
    print("Servo Current:  %f A (%fW)" % (e.servo_current, e.servo_current * e.system_voltage))
    print("Aux Current:    %f A (%fW)" % (e.aux_current, e.aux_current * e.system_voltage))
    print("Motor 1")
    print("  Velocity:     %d" % e.motor1_vel)
    print("  Position:     %d" % e.motor1_pos)
    print("  Current:      %d A" % e.motor1_current)
    print("  Gains:        Kp: %f, Kd: %f, Ki: %f, Windup: %f" %
          (e.motor1_kp, e.motor1_kd, e.motor1_ki, e.motor1_windup))
    print("Motor 2")
    print("  Velocity:     %d" % e.motor2_vel)
    print("  Position:     %d" % e.motor2_pos)
    print("  Current:      %d A" % e.motor2_current)
    print("  Gains:        Kp: %f, Kd: %f, Ki: %f, Windup: %f" %
          (e.motor2_kp, e.motor2_kd, e.motor2_ki, e.motor2_windup))
    print("Motor Period:   %d ms" % e.motor_period)
    print("Motor Max Step: %d" % e.motor_max_step)
    print("IMU")
    print("  Accel:        %d, %d, %d" % (e.accel_x, e.accel_y, e.accel_z))
    print("  Gyro:         %d, %d, %d" % (e.gyro_x, e.gyro_y, e.gyro_z))
    print("  Magnetometer: %d, %d, %d" % (e.mag_x, e.mag_y, e.mag_z))
    print("Tim12 Mode:     %d" % e.tim12_mode)
    if e.tim12_mode == 1:
        print("Tim12 Count:    %d" % e.tim12_count)


if __name__ == "__main__":
    main()
