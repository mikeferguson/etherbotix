#!/usr/bin/env python3

# Copyright (c) 2020 Michael Ferguson
# Copyright (c) 2018-2019 Botnuvo Inc
# All right reserved.

import argparse, struct
import matplotlib.pyplot as plot
from math import cos, sin, radians

from etherbotix_python.ax12 import AX_READ_DATA
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
                if packet == None or len(packet) == 0:
                    if len(pos) == 0:
                        print("No trace read - is firmware at least v3?")
                    # Done reading
                    break

                i = 0
                while i < len(packet):
                    p = struct.unpack_from("<i", packet, i+0)[0]
                    v = struct.unpack_from("<i", packet, i+4)[0]
                    s = struct.unpack_from("<i", packet, i+8)[0]
                    c = struct.unpack_from("<i", packet, i+12)[0]
                    i += 16
                    pos.append(p)
                    vel.append(v)
                    set.append(s)
                    cmd.append(c)

            if args.file:
                print("Saving to %s" % args.file)
                with open(args.file, "w") as f:
                    for i in range(len(th)):
                        f.write("%i,%i,%i,%i\n" % (pos[i], vel[i], set[i], cmd[i]))

            plot_trace(pos, vel, set, cmd)

if __name__ == "__main__":
    main()
