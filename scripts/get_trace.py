#!/usr/bin/env python3

# Copyright (c) 2020 Michael Ferguson
# Copyright (c) 2018-2019 Botnuvo Inc
# All right reserved.

import argparse, struct
import matplotlib.pyplot as plot
from math import cos, sin, radians

from etherbotix_python.ax12 import AX_READ_DATA
from etherbotix_python.etherbotix import Etherbotix

ADDR_TRACE = 194

if __name__ == "__main__":
    # Standard arguments
    parser = argparse.ArgumentParser(description="Read a motor trace from etherbotix.")
    parser.add_argument("--file", type=str, help="File to save trace to, or load from.")
    args = parser.parse_args()

    # Data
    pos = []
    vel = []
    cmd = []

    if args.file:
        # Load from file
        print("Loading from %s" % args.file)
        with open(args.file, "r") as f:
            line = f.readline().rstrip()
            while line:
                p, v, c = [int(x) for x in line.split(",")]

                pos.append(p)
                vel.append(v)
                cmd.append(c)

                line = f.readline().rstrip()

    else:
        print("Reading trace")

        e = Etherbotix()
        pkts = 0

        # Latch the trace
        e.write(253, ADDR_TRACE, [1])

        # Read out trace
        while (1):
            packet = e.read(253, ADDR_TRACE, 24)
            if packet == None or len(packet) == 0:
                # Done reading
                break
            print(pkts, packet)
            pkts += 1

            i = 0
            while i < len(packet):
                p = struct.unpack_from("<i", packet, i+0)[0]
                v = struct.unpack_from("<i", packet, i+4)[0]
                c = struct.unpack_from("<i", packet, i+8)[0]
                i += 12

                pos.append(p)
                vel.append(v)
                cmd.append(c)

        if args.file:
            print("Saving to %s" % args.file)
            with open(args.file, "w") as f:
                for i in range(len(th)):
                    f.write("%i,%i,%i\n" % (pos[i], vel[i], cmd[i]))

    # Plot Trace
    fig, plot1 = plot.subplots()
    plot1.plot(vel, label='vel', color='g')
    plot1.plot(cmd, label='cmd', color='b')
    plot2 = plot1.twinx()
    plot2.plot(pos, label='pos', color='r')
    plot.show()

