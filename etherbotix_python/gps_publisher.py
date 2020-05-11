#!/usr/bin/env python3

# Copyright (c) 2014-2020 Michael Ferguson
# Copyright (c) 2013 Vanadium Labs LLC.
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

import rclpy
from rclpy.node import Node

from nmea_msgs.msg import Sentence
from etherbotix_python.etherbotix import Etherbotix


class GPSPublisher(Node):
    """Publishes GPS sentences from USART3."""

    def __init__(self, ip="192.168.0.42", port=6707):
        super().__init__("gps_publisher")
        self.etherbotix = Etherbotix(ip, port)
        self.declare_parameter("frame_id", "base_link",)
        self.publisher = self.create_publisher(Sentence, "nmea_sentence", 10)

    def setup(self):
        # Set baud to 9600, set terminating character to '\n' (10)
        self.etherbotix.write(253, self.etherbotix.P_USART_BAUD, [207, 10])

    def run(self):
        self.setup()
        while True:
            packet = self.etherbotix.getPacket()
            if packet:
                s = Sentence()
                s.header.frame_id = self.get_parameter("frame_id").value
                s.header.stamp = self.get_clock().now().to_msg()
                s.sentence = str(packet.params.rstrip())
                self.publisher.publish(s)
                rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    g = GPSPublisher()
    try:
        g.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
