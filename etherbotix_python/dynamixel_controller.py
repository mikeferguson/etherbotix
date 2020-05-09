#!/usr/bin/env python3

# Copyright (c) 2014 Michael Ferguson
# Copyright (c) 2011-2013 Vanadium Labs LLC.
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

## @file dynamixel_controller.py Classes for servo interaction.

import rospy

from math import radians

from diagnostic_msgs.msg import *
from std_msgs.msg import Float64

from ax12 import *
from joints import *

from .etherbotix import *

class DynamixelServo(Joint):

    def __init__(self, node, name, ns="~joints"):
        Joint.__init__(self, node, name)
        n = ns+"/"+name+"/"

        self.id = int(rospy.get_param(n+"id"))
        self.ticks = rospy.get_param(n+"ticks", 1024)
        self.neutral = rospy.get_param(n+"neutral", self.ticks/2)
        if self.ticks == 4096:
            self.range = 360.0
        else:
            self.range = 300.0
        self.range = rospy.get_param(n+"range", self.range)
        self.rad_per_tick = radians(self.range)/self.ticks

        # TODO: load these from URDF
        self.max_angle = radians(rospy.get_param(n+"max_angle", self.range/2.0))
        self.min_angle = radians(rospy.get_param(n+"min_angle", -self.range/2.0))
        self.max_speed = radians(rospy.get_param(n+"max_speed", 684.0))
                                                # max speed = 114 rpm - 684 deg/s
        self.invert = rospy.get_param(n+"invert" ,False)
        self.readable = rospy.get_param(n+"readable", True)

        self.status = "OK"
        self.level = DiagnosticStatus.OK

        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = None                     # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.enabled = True                     # can we take commands?
        self.last = rospy.Time.now()

        self.reads = 0.0                        # number of reads
        self.errors = 0                         # number of failed reads
        self.total_reads = 0.0
        self.total_errors = [0.0]

        self.voltage = 0.0
        self.temperature = 0.0

        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def setCurrentFeedback(self, reading):
        """ Update angle in radians by reading from servo, or by
            using position passed in from a sync read (in ticks). """
        if reading > -1 and reading < self.ticks:     # check validity
            self.reads += 1
            self.total_reads += 1
            last_angle = self.position
            self.position = self.ticksToAngle(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logdebug("Invalid read of servo: id " + str(self.id) + ", value " + str(reading))
            self.errors += 1
            self.total_reads += 1
            return

    ## @brief Set the desired output postion.
    ## @param position The desired position (in radians).
    ## @returns The desired position, as converted to ticks.
    def setControlOutput(self, position):
        """ Set the position that controller is moving to.
            Returns output value in ticks. """
        if self.enabled:
            max_step = abs(self.max_speed / 5.)
            if position - self.position > max_step:
                self.desired = int(self.angleToTicks(self.position + max_step))
            elif self.position - position > max_step:
                self.desired = int(self.angleToTicks(self.position - max_step))
            else:
                self.desired = int(self.angleToTicks(position))
            # when simulating, need to set position here
            if self.node.sim:
                self.position = self.ticksToAngle(self.desired)
            return self.desired
        return -1

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        if self.temperature > 60:   # TODO: read this value from eeprom
            self.status = "OVERHEATED, SHUTDOWN"
            self.level = DiagnosticStatus.ERROR
        elif self.temperature > 50 and self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OVERHEATING"
            self.level = DiagnosticStatus.WARN
        elif self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OK"
            self.level = DiagnosticStatus.OK
        msg.level = self.level
        msg.message = self.status
        msg.values.append(KeyValue("Position", str(self.position)))
        msg.values.append(KeyValue("Temperature", str(self.temperature)))
        msg.values.append(KeyValue("Voltage", str(self.voltage)))
        if self.reads + self.errors > 100:
            self.total_errors.append((self.errors*100.0)/(self.reads+self.errors))
            if len(self.total_errors) > 10:
                self.total_errors = self.total_errors[-10:]
            self.reads = 0
            self.errors = 0
        msg.values.append(KeyValue("Reads", str(self.total_reads)))
        msg.values.append(KeyValue("Error Rate", str(sum(self.total_errors)/len(self.total_errors))+"%" ))
        if self.desired:
            msg.values.append(KeyValue("Torque", "ON"))
        else:
            msg.values.append(KeyValue("Torque", "OFF"))
        return msg

    def angleToTicks(self, angle):
        """ Convert an angle to ticks, applying limits. """
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    def ticksToAngle(self, ticks):
        """ Convert an ticks to angle, applying limits. """
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
        return angle

    def speedToTicks(self, rads_per_sec):
        """ Convert speed in radians per second to ticks, applying limits. """
        ticks = self.ticks * rads_per_sec / self.max_speed
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    ## @brief ROS callback for a Float64 command.
    def commandCb(self, req):
        if self.enabled:
            if self.controller and self.controller.active():
                # Under an action control, do not interfere
                return
            else:
                self.setControlOutput(req.data)

from controllers import *

class DynamixelController(Controller):

    def __init__(self, node, name):
        Controller.__init__(self, node, name)
        self.dynamixels = list()

        if not self.node.sim:
            # We create a new port here for our exclusive use
            self.private_etherbotix = Etherbotix(node.ip, node.port)

        for joint in node.joints.values():
            if isinstance(joint, DynamixelServo):
                self.dynamixels.append(joint)

        self.sync_delta = rospy.Duration(1.0/rospy.get_param("~sync_rate", 125.0))
        self.sync_next = rospy.Time.now() + self.sync_delta

    def update(self):
        if rospy.Time.now() > self.sync_next:
            # Send sync_write packet
            syncpkt = list()
            for joint in self.dynamixels:
                v = joint.desired
                if v != None:  # if was dirty
                    syncpkt.append([joint.id, int(v)%256, int(v)>>8])
            if not self.node.sim and len(syncpkt) > 0:
                self.private_etherbotix.syncWrite(P_GOAL_POSITION_L, syncpkt)

            # Send sync_read packet
            synclist = list()
            for joint in self.dynamixels:
                if joint.readable:
                    synclist.append(joint.id)
            if not self.node.sim and len(synclist) > 0:
                packet = self.private_etherbotix.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                if packet:
                    for joint in self.dynamixels:
                        try:
                            i = synclist.index(joint.id)*2
                            joint.setCurrentFeedback(packet.params_int[i]+(packet.params_int[i+1]<<8))
                        except IndexError:
                            # not a readable servo
                            continue

            # Wait for next time
            self.sync_next = rospy.Time.now() + self.sync_delta

    def getDiagnostics(self):
        # Update the servo voltages, temperatures
        if not self.node.sim:
            synclist = list()
            for joint in self.dynamixels:
                if joint.readable:
                    synclist.append(joint.id)
            if len(synclist) > 0:
                packet = self.private_etherbotix.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
                if packet:
                    for joint in self.dynamixels:
                        try:
                            i = synclist.index(joint.id)*2
                            if packet.params_int[i] < 250:
                                joint.voltage = packet.params_int[i]/10.0
                            if packet.params_int[i+1] < 100:
                                joint.temperature = packet.params_int[i+1]
                        except IndexError:
                            # not a readable servo
                            continue
        # No need to return diagnostics here, will be part of each servo
        return None
