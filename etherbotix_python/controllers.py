#!/usr/bin/env python

# Copyright (c) 2014 Michael E. Ferguson
# Copyright (c) 2010-2011 Vanadium Labs LLC.
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

# Author: Michael E. Ferguson

## @file controllers.py Base class and support functions for a controllers.

import rospy

## @brief Controllers interact with Etherbotix hardware.
class Controller:

    ## @brief Constructs a Controller instance.
    ## @param node The node instance.
    ## @param name The controller name.
    def __init__(self, node, name):
        self.name = name
        self.node = node

        # Get update rate
        desired_rate = rospy.get_param("~controllers/"+name+"/rate", -1)
        if desired_rate > 0:
            self.t_delta = rospy.Duration(1.0/desired_rate)
            self.t_next = rospy.Time.now() + self.t_delta
        else:
            self.t_next = None

        # Additional output for joint states publisher
        self.joint_names = list()
        self.joint_positions = list()
        self.joint_velocities = list()

    ## @brief Derived classes should put any hardware startup here.
    def startup(self):
        pass

    ## @brief Derived classes should put periodic update code here.
    def update(self):
        pass

    ## @brief Derived classes should put any hardware shutdown here.
    def shutdown(self):
        pass

    ## @brief Is the controller actively sending commands to joints?
    def active(self):
        return False

    ## @brief Get a diagnostics message for this joint.
    ## @return Diagnostics message.
    def getDiagnostics(self):
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg

    ## @brief Internal function that is called on startup.
    def execute_startup(self):
        if not self.node.sim:
            self.startup()

    ## @brief Internal function that is called on update.
    ## @param dt The timestep, as a rospy.Duration.
    def execute_update(self, dt):
        self.dt = dt
        if self.t_next:
            now = rospy.Time.now()
            if now > self.t_next - dt:
                self.update()
                self.t_next += self.t_delta
        else:
            # Update at full rate
            self.update()

    ## @brief Internal function that is called on shutdown.
    def execute_shutdown(self):
        if not self.node.sim:
            self.shutdown()

