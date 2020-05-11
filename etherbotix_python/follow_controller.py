# Copyright (c) 2014-2020 Michael E. Ferguson
# Copyright (c) 2011 Vanadium Labs LLC.
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

# Author: Michael Ferguson

import rospy
import actionlib

from scipy.interpolate import interp1d

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from etherbotix_python.controllers import Controller


class FollowController(Controller):
    """A controller for joint chains, exposing a FollowJointTrajectory action."""

    def __init__(self, node, name):
        Controller.__init__(self, node, name)
        self.interpolating = 0

        # Parameters: rates and joints
        ns = "~controllers/" + name + "/"
        self.rate = rospy.get_param(ns + "rate", 50.0)
        self.joints = rospy.get_param(ns + "joints")
        for joint in self.joints:
            self.node.joints[joint].controller = self

        # Action server
        name = rospy.get_param(ns + "action_name", 'follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction,
                                                   execute_cb=self.actionCb, auto_start=False)

        # Good old trajectory
        rospy.Subscriber(self.name + "/command", JointTrajectory, self.commandCb)
        self.splines = None

        rospy.loginfo("Started FollowController (" + self.name + "). Joints: " + str(self.joints))

    def startup(self):
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Joint names do not match action controlled joints."
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empty."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.setupTrajectory(traj):
            r = rospy.Rate(self.rate)
            while self.splines is not None:
                # TODO publish feedback
                r.sleep()
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution setup failed.")

        rospy.loginfo(self.name + ": Done.")

    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name + ": Received trajectory, but action is active")
            return
        self.setupTrajectory(msg)

    def setupTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)

        # Get indexes of joints
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()
        start_f = start.to_sec()
        end_f = start_f

        # Convert trajectory to splines
        splines = list()
        for i in indexes:
            # Create spline for each joint
            x = list()
            y = list()
            for point in traj.points:
                end = start_f + point.time_from_start.to_sec()
                if end > end_f:
                    end_f = end
                x.append(end)
                y.append(point.positions[i])
            splines.append(interp1d(x, y))
        self.end_f = end_f

        # Wait for splines to be valid
        while rospy.Time.now() + rospy.Duration(0.01) < start:
            rospy.sleep(0.01)

        # Ready to go
        self.splines = splines
        return True

    def update(self):
        if self.splines:
            # Have splines, are they still valid?
            now = rospy.Time.now().to_sec()
            if now > self.end_f:
                self.splines = None
                return
            # Splines are good, lets interpolate
            for i in range(len(self.splines)):
                joint = self.node.joints[self.joints[i]]
                joint.setControlOutput(self.splines[i](now))

    def active(self):
        """Is controller overriding servo internal control."""
        return self.splines is not None

    def getDiagnostics(self):
        """Get a diagnostics status."""
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg
