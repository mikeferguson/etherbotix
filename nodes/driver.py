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

# Author: Michael Ferguson

## @file driver.py Etherbotix ROS node.

import rospy

from etherbotix_python.etherbotix import *
from etherbotix_python.diff_controller import DiffController
from etherbotix_python.follow_controller import FollowController
from etherbotix_python.linear_controller import *
from etherbotix_python.dynamixel_controller import *
from etherbotix_python.publishers import *

# name: ControllerClass
controller_types = { "follow_controller" : FollowController,
                     "diff_controller"   : DiffController,
                     "linear_controller" : LinearControllerAbsolute,
                     "linear_controller_i" : LinearControllerIncremental }

class EtherbotixNode:

    def __init__(self):

        # Load config
        self.rate = float(rospy.get_param("~rate", 100.0))
        self.sim = rospy.get_param("~sim", False)
        if self.sim:
            rospy.loginfo("Etherbotix being simulated")
        else:
            self.ip = rospy.get_param("~ip", "192.168.0.42")
            self.port = int(rospy.get_param("~port", 6707))
            self.etherbotix = Etherbotix(self.ip, self.port)
            rospy.loginfo("Connecting to Etherbotix at " + self.ip + ":" + str(self.port))

        # Setup publishers
        self.diagnostics = DiagnosticsPublisher()
        self.joint_state_publisher = JointStatePublisher()
        if rospy.get_param("~imu", None):
            self.imu_publisher = ImuPublisher()
        else:
            self.imu_publisher = None

        # Setup joints
        self.joints = dict()
        for name in rospy.get_param("~joints", dict()).keys():
            joint_type = rospy.get_param("~joints/"+name+"/type", "dynamixel")
            if joint_type == "dynamixel":
                self.joints[name] = DynamixelServo(self, name)
            elif joint_type == "calibrated_linear":
                self.joints[name] = LinearJoint(self, name)

        # Setup controllers
        self.controllers = [DynamixelController(self, "servos"), ]
        controllers = rospy.get_param("~controllers", dict())
        for name, params in controllers.items():
            try:
                controller = controller_types[params["type"]](self, name)
                self.controllers.append(controller)
            except Exception as e:
                if type(e) == KeyError:
                    rospy.logerr("Unrecognized controller: " + params["type"])
                else:
                    rospy.logerr(str(type(e)) + str(e))
        for controller in self.controllers:
            controller.execute_startup()

        rospy.loginfo("Done initializing.")

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Update etherbotix state
            if not self.sim:
                pkt = self.etherbotix.execute(253, AX_READ_DATA, [0,128])
                if self.etherbotix.updateFromPacket(pkt):
                    if self.imu_publisher:
                        self.imu_publisher.publish(self.etherbotix)
                    # TODO: publish board state?

            # Update controllers
            for controller in self.controllers:
                controller.execute_update(rospy.Duration(1/self.rate))

            # Publish feedback
            self.joint_state_publisher.update(self.joints.values(), self.controllers)
            self.diagnostics.update(self.joints.values(), self.controllers, self.etherbotix)

            r.sleep()

        for controller in self.controllers:
            controller.shutdown()

if __name__ == "__main__":
    rospy.init_node("etherbotix")
    e = EtherbotixNode()
    e.run()
