#!/usr/bin/env python3

# Copyright (c) 2018-2020 Michael Ferguson
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
import curses
from etherbotix_python.ax12 import AX_READ_DATA
from etherbotix_python.etherbotix import Etherbotix

DEFAULT_OK = 1
DEFAULT_ERROR = 2
WHITE_OK = 3
WHITE_ERROR = 4


def main(args=None):
    e = Etherbotix()

    # Stats
    packets_sent = 0
    packets_ret = 0

    # Running average for currents
    servo_current = 0.0
    aux_current = 0.0

    try:
        # Setup full screen
        screen = curses.initscr()
        curses.noecho()
        curses.start_color()
        curses.curs_set(0)

        # Setup colors
        curses.init_pair(DEFAULT_OK, curses.COLOR_WHITE, curses.COLOR_BLACK)
        screen.bkgd(" ", curses.color_pair(DEFAULT_OK))
        curses.init_pair(DEFAULT_ERROR, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(WHITE_OK, curses.COLOR_BLUE, curses.COLOR_WHITE)
        curses.init_pair(WHITE_ERROR, curses.COLOR_RED, curses.COLOR_WHITE)

        while True:
            screen.addstr(1, 1, "Etherbotix Monitor                   ")
            screen.border()

            # Update from board
            pkt = e.execute(253, AX_READ_DATA, [0, 128])
            packets_sent += 1
            if pkt is None:
                screen.addstr(1, 19, " (CONNECTION LOST)")
            else:
                packets_ret += 1
                e.updateFromPacket(pkt)

                screen.addstr(3, 3, "Firmware Version: %d" % e.version)
                screen.addstr(4, 3, "System Time:      %d" % e.system_time)
                screen.addstr(5, 3, "System Voltage:   ")
                if (e.system_voltage < 11.0):
                    screen.addstr(5, 21, "%5.2fV" % e.system_voltage, curses.color_pair(DEFAULT_ERROR))  # noqa: E501
                else:
                    screen.addstr(5, 21, "%5.2fV" % e.system_voltage)
                servo_current = (0.7 * servo_current) + (0.3 * e.servo_current)
                aux_current = (0.7 * aux_current) + (0.3 * e.aux_current)
                screen.addstr(6, 3, "Servo Current:    %.2fA        " % servo_current)
                screen.addstr(7, 3, "Aux. Current:     %.2fA        " % aux_current)
                reliability = float(packets_ret) / float(packets_sent) * 100.0
                screen.addstr(9, 3, "Packets Sent:     %d" % packets_sent)
                screen.addstr(10, 3, "Packets Recv:     %d" % e.packets_recv)
                screen.addstr(11, 3, "Packets Returned: %d (%.2f%%)      " %
                              (packets_ret, reliability))
                screen.addstr(12, 3, "Packets Bad:      %d" % e.packets_bad)

                # Digital IO Status
                digital_dir = " ".join(["U" if e.user_io_use & (1 << n) else "A" if e.digital_dir & (1 << n) == 0 else "O" for n in range(3)]) + " "  # noqa: E501
                digital_dir += " ".join(["U" if e.user_io_use & (1 << (3 + n)) else "I" if e.digital_dir & (1 << (3 + n)) == 0 else "O" for n in range(5)])  # noqa: E501
                digital_in = " ".join(["L" if e.digital_in & (1 << n) == 0 else "H" for n in range(8)])  # noqa: E501
                screen.addstr(14, 3, "Digital    0 1 2 3 4 5 6 7")
                screen.addstr(15, 3, "Direction: %s" % digital_dir, curses.color_pair(WHITE_OK))
                screen.addstr(16, 3, "Value:     %s" % digital_in, curses.color_pair(WHITE_OK))

                # Motors & IMU in second column
                screen.addstr(1, 40, "Motor 1")
                screen.addstr(2, 40, "Position:      %15d" % e.motor1_pos, curses.color_pair(WHITE_OK))  # noqa: E501
                screen.addstr(3, 40, "Velocity:      %15d" % e.motor1_vel, curses.color_pair(WHITE_OK))  # noqa: E501
                screen.addstr(5, 40, "Motor 2")
                screen.addstr(6, 40, "Position:      %15d" % e.motor2_pos, curses.color_pair(WHITE_OK))  # noqa: E501
                screen.addstr(7, 40, "Velocity:      %15d" % e.motor2_vel, curses.color_pair(WHITE_OK))  # noqa: E501
                screen.addstr(9, 40, "IMU (v%d)  " % e.getImuVersion())
                if (e.imu_flags & 0xE0) != 0xE0:
                    screen.addstr(9, 65, "ERROR", curses.color_pair(DEFAULT_ERROR))
                else:
                    screen.addstr(9, 65, "     ", curses.color_pair(DEFAULT_ERROR))
                screen.addstr(10, 40, "Accel:  %6d  %6d  %6d" %
                              (e.accel_x, e.accel_y, e.accel_z), curses.color_pair(WHITE_OK))
                screen.addstr(11, 40, "Gyro:   %6d  %6d  %6d" %
                              (e.gyro_x, e.gyro_y, e.gyro_z), curses.color_pair(WHITE_OK))
                screen.addstr(12, 40, "Mag:    %6d  %6d  %6d" %
                              (e.mag_x, e.mag_y, e.mag_z), curses.color_pair(WHITE_OK))

                screen.addstr(14, 40, "Analog 0:      %15d" % e.a0, curses.color_pair(WHITE_OK))
                screen.addstr(15, 40, "Analog 1:      %15d" % e.a1, curses.color_pair(WHITE_OK))
                screen.addstr(16, 40, "Analog 2:      %15d" % e.a2, curses.color_pair(WHITE_OK))

            screen.refresh()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        curses.endwin()


if __name__ == "__main__":
    main()
