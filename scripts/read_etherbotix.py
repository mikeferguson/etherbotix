#!/usr/bin/env python

# Copyright (c) 2014 Michael Ferguson
# Copyright (c) 2008-2013 Vanadium Labs LLC.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time
from etherbotix.ax12 import *
from etherbotix.etherbotix import *

if __name__ == "__main__":
    e = Etherbotix()
    t = time.time()
    packets = 0
    while True:
        try:
            pkt = e.execute(253, AX_READ_DATA, [0,128])
            e.updateFromPacket(pkt)
            print(e.system_time, e.packets_recv, e.packets_bad, e.system_voltage)
            print(e.gyro_x)
            print(e.gyro_y)
            print(e.gyro_z)
            print(e.motor1_pos, e.motor2_pos)
            print("")
            packets += 1
            time.sleep(0.01)
        except KeyboardInterrupt:
            break

    print("Sent", packets, "in", time.time()-t, "seconds")
