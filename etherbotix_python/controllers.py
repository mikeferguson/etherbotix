# Copyright (c) 2014-2020 Michael Ferguson
# Copyright (c) 2010-2011 Vanadium Labs LLC.
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

# Author: Michael E. Ferguson

import rospy
from diagnostic_msgs.msg import DiagnosticStatus


class Controller:
    """Controllers interact with Etherbotix hardware."""

    def __init__(self, node, name):
        """
        Construct a Controller Instance.

        node -- The node instance
        name -- The controller name
        """
        self.name = name
        self.node = node

        # Get update rate
        ns = "~controllers" + name + "/"
        desired_rate = rospy.get_param(ns + "rate", -1)
        if desired_rate > 0:
            self.t_delta = rospy.Duration(1.0 / desired_rate)
            self.t_next = rospy.Time.now() + self.t_delta
        else:
            self.t_next = None

        # Additional output for joint states publisher
        self.joint_names = list()
        self.joint_positions = list()
        self.joint_velocities = list()

    def startup(self):
        """Put any hardware startup here."""
        pass

    def update(self):
        """Put periodic update code here."""
        pass

    def shutdown(self):
        """Put any hardware shutdown here."""
        pass

    def active(self):
        """Return whether controller actively sending commands."""
        return False

    def getDiagnostics(self):
        """Get a diagnostics message for this controller."""
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg

    def execute_startup(self):
        """Handle startup."""
        self.startup()

    def execute_update(self, dt):
        """Handle periodic update."""
        self.dt = dt
        if self.t_next:
            now = rospy.Time.now()
            if now > self.t_next - dt:
                self.update()
                self.t_next += self.t_delta
        else:
            # Update at full rate
            self.update()

    def execute_shutdown(self):
        """Handle shutdown."""
        self.shutdown()
