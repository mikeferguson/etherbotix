#!/usr/bin/env python

# Copyright (c) 2014-2020 Michael E. Ferguson
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

## @file diff_controller.py Differential drive controller for Etherbotix.

from math import sin, cos, pi

import rospy
from tf.broadcaster import TransformBroadcaster

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import *

from .ax12 import *
from .controllers import *

## @brief Used to compare floating point parameters
def almost_equal(f1, f2):
    diff = float(f1) - float(f2)
    if abs(diff) < 0.0001:
        return True
    return False

## @brief Controller to handle movement & odometry feedback for a differential drive mobile base.
class DiffController(Controller):

    def __init__(self, node, name):
        Controller.__init__(self, node, name)
        self.last_cmd = rospy.Time.now()

        # Parameters: command timeout
        self.timeout = rospy.get_param("~controllers/"+name+"/timeout", 0.5)

        # Parameters: geometry
        self.ticks_meter = float(rospy.get_param("~controllers/"+name+"/ticks_meter"))
        self.base_width = float(rospy.get_param("~controllers/"+name+"/base_width"))

        # This is the wheel rollout - ticks/revolution - used for joint_states publisher
        # ticks_meter is clearly wrong, but was the old behavior
        self.ticks_rotation = float(rospy.get_param("~controllers/"+name+"/ticks_rotation", self.ticks_meter))
        # One rotation is 2PI radians
        self.ticks_rotation /= 2 * pi

        self.base_frame_id = rospy.get_param("~controllers/"+name+"/base_frame_id", "base_link")
        self.odom_frame_id = rospy.get_param("~controllers/"+name+"/odom_frame_id", "odom")

        # Parameters: PID
        self.Kp = rospy.get_param("~controllers/"+name+"/Kp", 1.0)
        self.Kd = rospy.get_param("~controllers/"+name+"/Kd", 0.0)
        self.Ki = rospy.get_param("~controllers/"+name+"/Ki", 0.1)
        self.Kw = rospy.get_param("~controllers/"+name+"/Kw", 400.0)

        # Parameters: motor period
        self.period = rospy.get_param("~controllers/"+name+"/period", 10.0)

        # Parameters: acceleration
        self.accel_limit = rospy.get_param("~controllers/"+name+"/accel_limit", 0.1)

        # Parameters: twist covariance
        self.covariance_vx = rospy.get_param("~controllers/"+name+"/covariance_vx", 1e-3)
        self.covariance_vy = rospy.get_param("~controllers/"+name+"/covariance_vy", 1e-3)
        self.covariance_vz = rospy.get_param("~controllers/"+name+"/covariance_vz", 1e-6)
        self.covariance_wx = rospy.get_param("~controllers/"+name+"/covariance_wx", 1e-6)
        self.covariance_wy = rospy.get_param("~controllers/"+name+"/covariance_wy", 1e-6)
        self.covariance_wz = rospy.get_param("~controllers/"+name+"/covariance_wz", 1e-3)

        # Output for joint states publisher
        left_joint = rospy.get_param("~controllers/"+name+"/left_joint_name", "base_l_wheel_joint")
        right_joint = rospy.get_param("~controllers/"+name+"/right_joint_name", "base_r_wheel_joint")
        self.joint_names = [left_joint, right_joint]
        self.joint_positions = [0.0, 0.0]
        self.joint_velocities = [0.0, 0.0]
        # Support for 4WD bases where the each side motor pair has a single encoder
        if rospy.has_param("~controllers/"+name+"/left_joint_mimic") and \
           rospy.has_param("~controllers/"+name+"/right_joint_mimic"):
            left_mimic_joint = rospy.get_param("~controllers/"+name+"/left_joint_mimic", "base_lr_wheel_joint")
            right_mimic_joint = rospy.get_param("~controllers/"+name+"/right_joint_mimic", "base_rr_wheel_joint")
            self.joint_names.extend([left_mimic_joint, right_mimic_joint])
            self.joint_positions.extend([0.0, 0.0])
            self.joint_velocities.extend([0.0, 0.0])

        # Internal data
        self.v_left = 0                 # current setpoint velocity
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # Publish odometry, optionally TF
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.odomBroadcaster = None
        if rospy.get_param("~controllers/"+name+"/publish_tf", True):
            self.odomBroadcaster = TransformBroadcaster()

        # Subscribe to command
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)

        rospy.loginfo("Started DiffController ("+name+").")
        rospy.loginfo("  Geometry: " + str(self.base_width) + "m wide, " + str(self.ticks_meter) + " ticks/m.")

    def update(self):
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        # Do odometry first
        if self.node.sim:
            # If simulated, forward simulate trajectory
            x = cos(self.th)*self.dx*elapsed
            y = -sin(self.th)*self.dx*elapsed
            self.x += cos(self.th)*self.dx*elapsed
            self.y += sin(self.th)*self.dx*elapsed
            self.th += self.dr*elapsed
        else:
            try:
                left = self.node.etherbotix.motor1_pos
                right = self.node.etherbotix.motor2_pos
            except AttributeError:
                # board is not yet updated
                return False

            # calculate position
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (left - self.enc_left)/self.ticks_meter
                d_right = (right - self.enc_right)/self.ticks_meter
            self.enc_left = left
            self.enc_right = right

            d = (d_left+d_right)/2
            th = (d_right-d_left)/self.base_width

            # calculate velocity
            l_vel = self.node.etherbotix.motor1_vel / self.ticks_meter
            r_vel = self.node.etherbotix.motor2_vel / self.ticks_meter
            self.dx = (l_vel+r_vel)/2 * (1000.0/self.period)
            self.dr = (r_vel-l_vel)/self.base_width * (1000.0/self.period)

            if (d != 0):
                x = cos(th)*d
                y = -sin(th)*d
                self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
                self.y = self.y + (sin(self.th)*x + cos(self.th)*y)
            if (th != 0):
                self.th = self.th + th

            # Update joint_states publisher
            self.joint_positions = [self.enc_left/self.ticks_rotation, self.enc_right/self.ticks_rotation]
            self.joint_velocities = [l_vel * (1000.0/self.period), r_vel * (1000.0/self.period)]
            if len(self.joint_names) == 4:
                self.joint_positions.extend(self.joint_positions)
                self.joint_velocities.extend(self.joint_velocities)

        # Publish or perish
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)
        if self.odomBroadcaster:
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        odom.twist.covariance[0] = self.covariance_vx
        odom.twist.covariance[7] = self.covariance_vy
        odom.twist.covariance[14] = self.covariance_vz
        odom.twist.covariance[21] = self.covariance_wx
        odom.twist.covariance[28] = self.covariance_wy
        odom.twist.covariance[35] = self.covariance_wz
        self.odomPub.publish(odom)

        # Now update commands
        if now > (self.last_cmd + rospy.Duration(self.timeout)):
            self.v_des_left = 0
            self.v_des_right = 0

        # Update motors if real hardware and PID is correct
        if not self.node.sim and self.updateParams():
            max_accel = int(self.accel_limit * self.ticks_meter * self.dt.to_sec())

            # Limit left side acceleration
            if self.v_left < self.v_des_left:
                self.v_left += max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left

            # Limit right side acceleration
            if self.v_right < self.v_des_right:
                self.v_right += max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right

            # Send commands
            self.updateControls(self.v_left, self.v_right)

    ## @brief On shutdown, need to stop base.
    def shutdown(self):
        self.updateControls(0,0)

    ## @brief ROS callback to set new velocity.
    ## @param req A geometry_msgs/Twist command.
    def cmdVelCb(self, req):
        # set motor speeds in ticks per period
        self.v_des_left = int( ((req.linear.x - (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / (1000.0/self.period))
        self.v_des_right = int( ((req.linear.x + (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / (1000.0/self.period))
        self.last_cmd = rospy.Time.now()

    ## @brief Get diagnostics message.
    def getDiagnostics(self):
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if not self.node.sim:
            msg.values.append(KeyValue("Left", str(self.enc_left)))
            msg.values.append(KeyValue("Right", str(self.enc_right)))
        msg.values.append(KeyValue("dX", str(self.dx)))
        msg.values.append(KeyValue("dR", str(self.dr)))
        return msg

    ## @brief Make sure the parameters on robot match our params.
    ## @returns True if parameters match.
    def updateParams(self):
        # Need to check PID params
        if not almost_equal(self.node.etherbotix.motor1_kp, self.Kp) or \
           not almost_equal(self.node.etherbotix.motor1_kd, self.Kd) or \
           not almost_equal(self.node.etherbotix.motor1_ki, self.Ki) or \
           not almost_equal(self.node.etherbotix.motor1_windup, self.Kw) or \
           not almost_equal(self.node.etherbotix.motor2_kp, self.Kp) or \
           not almost_equal(self.node.etherbotix.motor2_kd, self.Kd) or \
           not almost_equal(self.node.etherbotix.motor2_ki, self.Ki) or \
           not almost_equal(self.node.etherbotix.motor2_windup, self.Kw):
            params = struct.pack("<ffff", self.Kp, self.Kd, self.Ki, self.Kw)
            params = params + params # both sides are the same
            params = [ord(x) for x in params]
            self.node.etherbotix.write(253, self.node.etherbotix.P_MOTOR1_KP, params, ret=False)
            return False
        # Need to check motor period
        if self.node.etherbotix.motor_period != self.period:
            self.node.etherbotix.write(253, self.node.etherbotix.P_MOTOR_PERIOD, [self.period,] , ret=False)
            return False
        # Params are up to date
        return True

    ## @brief Update controls
    def updateControls(self, left, right):
        params = struct.pack("<hh", left, right)
        params = [ord(x) for x in params]
        self.node.etherbotix.write(253, self.node.etherbotix.P_MOTOR1_VEL, params, ret=False)
