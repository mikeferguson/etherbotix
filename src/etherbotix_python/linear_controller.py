#!/usr/bin/env python

# Copyright (c) 2014 Michael E. Ferguson
# Copyright (c) 2011 Vanadium Labs LLC.
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

## @file linear_controller.py Controllers for linear actuators (with analog
##       or optical encoder feedback)

import rospy, actionlib

from joints import *
from controllers import *
from std_msgs.msg import Float64
from diagnostic_msgs.msg import *
from std_srvs.srv import *

from struct import unpack

class LinearJoint(Joint):
    def __init__(self, node, name, ns="~joints"):
        Joint.__init__(self, node, name)
        n = ns+"/"+name+"/"

        self.position = 0.0                     # current position, as returned by feedback (meters)
        self.desired = None                     # desired position (meters)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()

        # TODO: load these from URDF
        self.min_pos = rospy.get_param(n+"min_position",0.0)
        self.max_pos = rospy.get_param(n+"max_position",0.5)
        self.max_speed = rospy.get_param(n+"max_speed",0.0508)

        # Calibration data {reading: position}
        self.cal = { -1: -1, 1: 1 }
        self.cal_raw = rospy.get_param(n+"calibration_data", self.cal)
        self.cal = dict()
        for key, value in self.cal_raw.items():
            self.cal[int(key)] = value
        self.keys = sorted(self.cal.keys())

        rospy.Subscriber(name+"/command", Float64, self.commandCb)

    def setCurrentFeedback(self, reading):
        if reading >= self.keys[0] and reading <= self.keys[-1]:
            last_angle = self.position
            self.position = self.readingToPosition(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logerr(self.name + ": feedback reading out of range")

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in raw_data format. """
        if position <= self.max_pos and position >= self.min_pos:
            self.desired = position
        else:
            rospy.logerr(self.name + ": requested position is out of range: " + str(position))
        return None # TODO
    
    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        if self.desired != None:
            msg.message = "Moving"
        else:
            msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.node.sim:
            self.position = req.data
        else:
            self.setControlOutput(req.data)

    def readingToPosition(self, reading):
        low = 0
        while reading > self.keys[low+1]:
            low += 1
        high = len(self.keys) - 1
        while reading < self.keys[high-1]:
            high += -1
        x = self.keys[high] - self.keys[low]
        y = self.cal[self.keys[high]] - self.cal[self.keys[low]]
        x1 = reading - self.keys[low]
        y1 = y * ( float(x1)/float(x) )
        return self.cal[self.keys[low]] + y1


class LinearControllerAbsolute(Controller):
    """ A controller for a linear actuator, with absolute positional feedback. """

    def __init__(self, node, name):
        Controller.__init__(self, node, name)

        self.a = rospy.get_param("~controllers/"+name+"/motor_a", 0)
        self.b = rospy.get_param("~controllers/"+name+"/motor_b", 1)
        self.p = rospy.get_param("~controllers/"+name+"/motor_pwm",7)
        self.analog = rospy.get_param("~controllers/"+name+"/feedback",0)
        self.last_reading = 0
        self.last_speed = 0

        self.joint = node.joints[rospy.get_param("~controllers/"+name+"/joint")]

        rospy.loginfo("Started LinearController ("+self.name+").")

    def startup(self):
        if not self.node.sim:
            self.joint.setCurrentFeedback(self.device.getAnalog(self.analog))

    def update(self):
        now = rospy.Time.now()
        if not self.node.sim:
            # Read current position
            try:
                self.last_reading = self.getPosition()
                self.joint.setCurrentFeedback(self.last_reading)
            except Exception as e:
                print "linear error: ", e
            # Update movement
            if self.joint.desired != None:
                if self.joint.desired > self.joint.position:
                    self.setSpeed(1)
                elif self.joint.desired < self.joint.position:
                    self.setSpeed(-1)
                else:
                    self.setSpeed(0)
                    self.joint.desired = None

    def setSpeed(self, speed):
        """ Set speed of actuator. """
        self.last_speed = speed
        if speed > 0:
            self.node.etherbotix.setDigital(self.a, 1); self.node.etherbotix.setDigital(self.b, 0);  # up
            self.node.etherbotix.setDigital(self.p, 1)
        elif speed < 0:
            self.node.etherbotix.setDigital(self.a, 0); self.node.etherbotix.setDigital(self.b, 1);  # down
            self.node.etherbotix.setDigital(self.p, 1)
        else:
            self.node.etherbotix.setDigital(self.p, 0)

    def getPosition(self):
        return self.node.etherbotix.getAnalog(self.analog)

    def shutdown(self):
        if not self.node.sim:
            self.node.etherbotix.setDigital(self.p, 0)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name

        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if not self.node.sim:
            msg.values.append(KeyValue("Encoder Reading", str(self.last_reading)))
        
        return msg


class LinearControllerIncremental(LinearControllerAbsolute):
    """ A controller for a linear actuator, without absolute encoder. """

    def __init__(self, node, name):
        Controller.__init__(self, node, name)
        self.pause = True

        self.a = rospy.get_param("~controllers/"+name+"/motor_a", 0)
        self.b = rospy.get_param("~controllers/"+name+"/motor_b", 1)
        self.p = rospy.get_param("~controllers/"+name+"/motor_pwm",7)
        self.last_raw = 0
        self.last_reading = 0
        self.last_speed = 0

        self.joint = node.joints[rospy.get_param("~controllers/"+name+"/joint")]

        # Need to track real position
        self.position = 0  # virtual position 0->X

        rospy.Service(name+"/zero", Empty, self.zeroCb)
        rospy.loginfo("Started LinearControllerIncremental ("+self.name+").")

    def startup(self):
        if not self.node.sim:
            self.node.etherbotix.setTimer12Mode(1)
            self.zeroEncoder(get_latest=True)

    def update(self):
        now = rospy.Time.now()
        if not self.node.sim:
            # Read current position
            try:
                self.last_reading = self.getPosition()
                self.joint.setCurrentFeedback(self.last_reading)
            except Exception as e:
                return
            # Update movement
            if self.joint.desired != None:
                if self.joint.desired > self.joint.position + self.getStepSize():
                    self.setSpeed(1)
                elif self.joint.desired < self.joint.position - self.getStepSize():
                    self.setSpeed(-1)
                else:
                    self.setSpeed(0)
                    self.joint.desired = None

    def getPosition(self):
        # The raw value only ever goes up, regardless of direction of
        # motion. We therefore have to apply our own direction value
        # to the measured difference in position
        raw = self.node.etherbotix.tim12_count
        diff = raw - self.last_raw
        self.last_raw = raw
        self.position += self.last_speed * diff
        if self.position < self.joint.keys[0]:
            self.position = self.joint.keys[0]
        if self.position > self.joint.keys[-1]:
            self.position = self.joint.keys[-1]
        return self.position

    def zeroEncoder(self, timeout=15.0, get_latest=False):
        rospy.loginfo(self.name + ": zeroing encoder")
        self.setSpeed(1)
        last_pos = None
        for i in range(int(timeout)):
            if rospy.is_shutdown():
                return
            try:
                if get_latest:
                    # When starting up, we need to get latest value
                    # since main loop will not be running
                    self.node.etherbotix.getTimer12Count()
                new_pos = self.node.etherbotix.tim12_count
            except:
                continue
            if last_pos == new_pos:
                break
            last_pos = new_pos
            rospy.sleep(1)
        self.setSpeed(0)
        self.position = 0
        self.last_raw = last_pos
        self.joint.setCurrentFeedback(self.position)

    def zeroCb(self, msg):
        if not self.node.sim:
            self.zeroEncoder(15.0)
        return EmptyResponse()

    def shutdown(self):
        if not self.node.sim:
            self.setSpeed(0)

    def getStepSize(self):
        try:
            return abs(self.joint.readingToPosition(1) - self.joint.readingToPosition(0))
        except:
            return abs(self.joint.readingToPosition(-1) - self.joint.readingToPosition(0))
