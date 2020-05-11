#!/usr/bin/env python3

# Copyright (c) 2020 Michael Ferguson
# Copyright (c) 2018-2019 Botnuvo Inc
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

import argparse
import struct
import matplotlib.pyplot as plot

from etherbotix_python.etherbotix import Etherbotix


def plot_trace(pos, vel, set, cmd):
    # Plot Trace
    fig, plot1 = plot.subplots()
    plot1.plot(vel, label='vel', color='r')
    plot1.plot(set, label='setpoint', color='g')
    plot1.plot(cmd, label='cmd', color='b')
    plot2 = plot1.twinx()
    plot2.plot(pos, label='pos', color='r')
    plot.show()


def main(args=None):
    # Standard arguments
    parser = argparse.ArgumentParser(description="Read a motor trace from etherbotix.")
    parser.add_argument("--left", action="store_true", help="Read a trace from left motor.")
    parser.add_argument("--right", action="store_true", help="Read a trace from right motor.")
    parser.add_argument("--file", type=str, help="File to save trace to, or load from.")
    args = parser.parse_args()

    if args.file:
        # Load from file
        print("Loading from %s" % args.file)

        # Data
        pos = []
        vel = []
        set = []
        cmd = []

        with open(args.file, "r") as f:
            line = f.readline().rstrip()
            while line:
                p, v, s, c = [int(x) for x in line.split(",")]

                pos.append(p)
                vel.append(v)
                set.append(s)
                cmd.append(c)

                line = f.readline().rstrip()

        plot_trace(pos, vel, set, cmd)

    else:
        e = Etherbotix()
        traces = []

        if args.left:
            # Latch a trace on left motor
            e.write(253, e.P_DEVICE_M1_TRACE, [1])
            traces.append(e.P_DEVICE_M1_TRACE)
            print("Latched a trace from left motor")
        if args.right:
            # Latch a trace on right motor
            e.write(253, e.P_DEVICE_M2_TRACE, [1])
            traces.append(e.P_DEVICE_M2_TRACE)
            print("Latched a trace from right motor")

        # Read out trace(s)
        for addr in traces:
            # Data
            pos = []
            vel = []
            set = []
            cmd = []

            while (1):
                packet = e.read(253, addr, 128)
                if packet is None or len(packet) == 0:
                    if len(pos) == 0:
                        print("No trace read - is firmware at least v3?")
                    # Done reading
                    break

                i = 0
                while i < len(packet):
                    p = struct.unpack_from("<i", packet, i + 0)[0]
                    v = struct.unpack_from("<i", packet, i + 4)[0]
                    s = struct.unpack_from("<i", packet, i + 8)[0]
                    c = struct.unpack_from("<i", packet, i + 12)[0]
                    i += 16
                    pos.append(p)
                    vel.append(v)
                    set.append(s)
                    cmd.append(c)

            if args.file:
                print("Saving to %s" % args.file)
                with open(args.file, "w") as f:
                    for i in range(len(pos)):
                        f.write("%i,%i,%i,%i\n" % (pos[i], vel[i], set[i], cmd[i]))

            plot_trace(pos, vel, set, cmd)


if __name__ == "__main__":
    main()
