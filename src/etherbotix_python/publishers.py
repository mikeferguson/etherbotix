#!/usr/bin/env python

# Copyright (c) 2014-2018 Michael Ferguson
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

## @file publishers.py Publishers of ROS data.

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState, Imu

## @brief Publishes diagnostics at some update rate
class DiagnosticsPublisher:

    def __init__(self):
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~diagnostic_rate", 1.0))
        self.t_next = rospy.Time.now() + self.t_delta
        self.pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=5)

    def update(self, joints, controllers, etherbotix = None):
        now = rospy.Time.now()
        if now > self.t_next:
            # Create message
            msg = DiagnosticArray()
            msg.header.stamp = now
            # Update controllers first, since dynamixel controller will update joints
            for controller in controllers:
                d = controller.getDiagnostics()
                if d:
                    msg.status.append(d)
            for joint in joints:
                d = joint.getDiagnostics()
                if d:
                    msg.status.append(d)
            # If we have Etherbotix, create diagnostics
            if etherbotix and etherbotix.system_time > 0:
                msg.status.append(self.getEtherbotixDiagnostics(etherbotix))
            # Publish and update stats
            self.pub.publish(msg)
            self.t_next = now + self.t_delta

    ## @brief Diagnostics generation for Etherbotix.
    ##        Located here since etherbotix.py is ROS-free.
    def getEtherbotixDiagnostics(self, etherbotix):
        msg = DiagnosticStatus()
        msg.name = "etherbotix"
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        msg.hardware_id = etherbotix.getUniqueId()
        # System Voltage
        if etherbotix.system_voltage < 10.0:
            msg.level = DiagnosticStatus.ERROR
            msg.message = "Battery depleted!"
        msg.values.append(KeyValue("Voltage", str(etherbotix.system_voltage)+"V"))
        # Currents
        msg.values.append(KeyValue("Servo Current", str(etherbotix.servo_current)+"A"))
        msg.values.append(KeyValue("Aux. Current", str(etherbotix.aux_current)+"A"))
        msg.values.append(KeyValue("Packets", str(etherbotix.packets_recv)))
        msg.values.append(KeyValue("Packets Bad", str(etherbotix.packets_bad)))
        return msg

## @brief Publishes joint_state messages on every update.
class JointStatePublisher:

    def __init__(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def update(self, joints, controllers):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        for joint in joints:
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
        for controller in controllers:
            msg.name += controller.joint_names
            msg.position += controller.joint_positions
            msg.velocity += controller.joint_velocities
        self.pub.publish(msg)

## @brief Publisher of IMU data
class ImuPublisher:

    def __init__(self):
        # Determine which version of IMU we are using
        self.version = 0

        # Name of tf frame to associate data with
        self.frame = rospy.get_param("~imu/frame_id", "imu_link")

        # Add publishers
        self._pub = rospy.Publisher("imu_raw", Imu, queue_size=5)
        self._pub_mag = None
        if rospy.get_param("~imu/publish_mag", True):
            self._pub_mag = rospy.Publisher("mag_raw", Vector3Stamped, queue_size=5)

    def publish(self, etherbotix):
        # Can't publish yet if don't know the version of our IMU
        if etherbotix.version > 0 and etherbotix.getImuVersion() == 0:
           return

        # Need to read params first time we get version
        if self.version == 0:
            # Save IMU version so we only run this once
            self.version = etherbotix.getImuVersion()

            # Backwards compatability with old firmware
            if etherbotix.version == 0:
                self.version = 2
                # L3GD20 (2000dps mode)
                self.gyro_scale = 0.001221111
                self.gyro_covariance = 0.004868938
                # LSM303DLHC (2g/full scale mode)
                self.accel_scale = 0.000598773
                self.accel_covariance = 0.34644996
                # LSM303DLHC (
                self.mag_scale = 0.000909

            # Set default params
            if self.version == 2:
                # L3GD20 (2000dps mode)
                self.gyro_scale = 0.001221111
                self.gyro_covariance = 0.004868938
                # LSM303DLHC (8g/full scale mode)
                self.accel_scale = 0.0023957
                self.accel_covariance = 0.34644996
                # LSM303DLHC (8.1 gauss mode)
                self.mag_scale = 0.0043478
            elif self.version == 3:
                # L3GD20H (2000dps mode)
                self.gyro_scale = 0.001221111
                self.gyro_covariance = 0.004868938
                # LSM303D (8g/full scale mode)
                self.accel_scale = 0.0023964
                self.accel_covariance = 0.34644996
                # LSM303D (12 gauss mode)
                self.mag_scale = 0.000479
            elif self.version == 5:
                # LSM6DS33 (2000dps mode)
                self.gyro_scale = 0.001221111
                self.gyro_covariance = 0.004868938
                # LSM6DS33 (8g/full scale mode)
                self.accel_scale = 0.0023964
                self.accel_covariance = 0.34644996
                # LIS3MDL (12 gauss mode)
                self.mag_scale = 0.000438404

            self.gyro_scale = rospy.get_param("~imu/gyro/scale", self.gyro_scale)
            self.gyro_covariance = rospy.get_param("~imu/gyro/covariance", self.gyro_covariance)

            self.accel_scale = rospy.get_param("~imu/accel/scale", self.accel_scale)
            self.accel_covariance = rospy.get_param("~imu/accel/covariance", self.accel_covariance)

            self.mag_scale = rospy.get_param("~imu/mag/scale", self.mag_scale)

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame

        # No known orientation
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = etherbotix.gyro_x * self.gyro_scale
        msg.angular_velocity.y = etherbotix.gyro_y * self.gyro_scale
        msg.angular_velocity.z = etherbotix.gyro_z * self.gyro_scale
        msg.angular_velocity_covariance[0] = self.gyro_covariance
        msg.angular_velocity_covariance[4] = self.gyro_covariance
        msg.angular_velocity_covariance[8] = self.gyro_covariance

        msg.linear_acceleration.x = etherbotix.accel_x * self.accel_scale
        msg.linear_acceleration.y = etherbotix.accel_y * self.accel_scale
        msg.linear_acceleration.z = etherbotix.accel_z * self.accel_scale
        msg.linear_acceleration_covariance[0] = self.accel_covariance
        msg.linear_acceleration_covariance[4] = self.accel_covariance
        msg.linear_acceleration_covariance[8] = self.accel_covariance

        self._pub.publish(msg)

        if self._pub_mag:
            mag = Vector3Stamped()
            mag.header = msg.header
            mag.vector.x = etherbotix.mag_x * self.mag_scale
            mag.vector.y = etherbotix.mag_y * self.mag_scale
            mag.vector.z = etherbotix.mag_z * self.mag_scale
            self._pub_mag.publish(mag)
