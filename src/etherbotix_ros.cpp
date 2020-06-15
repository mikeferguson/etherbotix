/*
 * Copyright 2013-2020 Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * based on http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/tutorial.html
 *      and http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/tutorial/tuttimer3.html
 */

#include <string>

#include "etherbotix/etherbotix_ros.hpp"
#include "etherbotix/copy_float.hpp"
#include "etherbotix/dynamixel.hpp"

namespace etherbotix
{

EtherbotixROS::EtherbotixROS(const rclcpp::NodeOptions & options)
: rclcpp::Node("etherbotix", options),
  Etherbotix(),
  logger_(rclcpp::get_logger("etherbotix")),
  initialized_(false)
{
  // Declare parameters
  ip_ = this->declare_parameter<std::string>("ip_address", ip_);
  port_ = this->declare_parameter<int>("port", port_);
  RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);

  milliseconds_ = this->declare_parameter<int>("update_interval_ms", 10);

  bool use_imu = this->declare_parameter<bool>("imu", true);
  imu_frame_id_ = this->declare_parameter<std::string>("imu.frame_id", "imu_link");

  // These default to 0.0 - if not set by user they will be set
  // later when we know the version of IMU on the board.
  gyro_scale_ = this->declare_parameter<double>("imu.gyro.scale", 0.0);
  gyro_covariance_ = this->declare_parameter<double>("imu.gyro.covariance", 0.0);
  accel_scale_ = this->declare_parameter<double>("imu.accel.scale", 0.0);
  accel_covariance_ = this->declare_parameter<double>("imu.accel.covariance", 0.0);
  mag_scale_ = this->declare_parameter<double>("imu.mag.scale", 0.0);

  // Setup motors
  double ticks_per_radian = this->declare_parameter<double>("ticks_per_radian", 1.0);
  left_motor_->set_ticks_per_radian(ticks_per_radian);
  right_motor_->set_ticks_per_radian(ticks_per_radian);

  double kp = this->declare_parameter<double>("motor_kp", 1.0);
  double kd = this->declare_parameter<double>("motor_kd", 0.0);
  double ki = this->declare_parameter<double>("motor_ki", 0.1);
  double kw = this->declare_parameter<double>("motor_kw", 400.0);
  RCLCPP_INFO(logger_, "Setting gains to %f %f %f %f", kp, kd, ki, kw);
  left_motor_->set_gains(kp, kd, ki, kw);
  right_motor_->set_gains(kp, kd, ki, kw);

  // Controller manager
  controller_manager_.reset(new robot_controllers_interface::ControllerManager());
  robot_controllers_interface::JointHandlePtr j;
  j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(left_motor_);
  controller_manager_->addJointHandle(j);
  j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(right_motor_);
  controller_manager_->addJointHandle(j);

  // ROS interfaces
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                           std::bind(&EtherbotixROS::publish, this));
  if (use_imu)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/raw", 10);
  }
}

EtherbotixROS::~EtherbotixROS()
{
}

void EtherbotixROS::update(const boost::system::error_code & e)
{
  // Abort if we are shutting down
  if (!rclcpp::ok())
  {
    return;
  }

  if (!initialized_)
  {
    // This has to be done here, shared_from_this doesn't work in constructor
    controller_manager_->init(shared_from_this());
    initialized_ = true;
  }

  // Reset joint handle commands
  left_motor_->reset();
  right_motor_->reset();

  // Update controllers
  uint64_t nanoseconds = milliseconds_ * 1e6;
  controller_manager_->update(this->now(), rclcpp::Duration(nanoseconds));

  // Generate Commands
  size_t length = 0;
  uint8_t send_buf[4096];

  send_buf[length++] = 0xff;
  send_buf[length++] = 'B';
  send_buf[length++] = 'O';
  send_buf[length++] = 'T';

  // Read 128 bytes from etherbotix
  length += dynamixel::get_read_packet(&send_buf[length], ETHERBOTIX_ID, 0, 128);

  // Get motor update packets
  length += right_motor_->get_packets(&send_buf[length], 2);
  length += left_motor_->get_packets(&send_buf[length], 1);

  this->send(send_buf, length);

  // Reset timer and async wait
  Etherbotix::update(e);
}

void EtherbotixROS::publish()
{
  if (!rclcpp::ok())
  {
    return;
  }

  auto now = this->now();

  // Publish joint states
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now;

  msg.name.push_back(left_motor_->getName());
  msg.position.push_back(left_motor_->getPosition());
  msg.velocity.push_back(left_motor_->getVelocity());
  msg.effort.push_back(left_motor_->getEffort());

  msg.name.push_back(right_motor_->getName());
  msg.position.push_back(right_motor_->getPosition());
  msg.velocity.push_back(right_motor_->getVelocity());
  msg.effort.push_back(right_motor_->getEffort());

  joint_state_pub_->publish(msg);

  // Publish IMU data
  if (imu_pub_ && this->get_imu_version() > 0)
  {
    if (gyro_scale_ == 0.0)
    {
      if (this->get_imu_version() == 2)
      {
        // L3GD20 (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM303DLHC (8g/full scale mode)
        accel_scale_ = 0.0023957;
        accel_covariance_ = 0.34644996;
        // LSM303DLHC (8.1 gauss mode)
        mag_scale_ = 0.0043478;
      }
      else if (this->get_imu_version() == 3)
      {
        // L3GD20H (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM303D (8g/full scale mode)
        accel_scale_ = 0.0023964;
        accel_covariance_ = 0.34644996;
        // LSM303D (12 gauss mode)
        mag_scale_ = 0.000479;
      }
      else  // assuming version == 5
      {
        // LSM6DS33 (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM6DS33 (8g/full scale mode)
        accel_scale_ = 0.0023964;
        accel_covariance_ = 0.34644996;
        // LIS3MDL (12 gauss mode)
        mag_scale_ = 0.000438404;
      }
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.header.stamp = now;

    // Signal no known orientation
    imu_msg.orientation_covariance[0] = -1.0;

    imu_msg.angular_velocity.x = this->get_imu_gyro_x() * gyro_scale_;
    imu_msg.angular_velocity.y = this->get_imu_gyro_y() * gyro_scale_;
    imu_msg.angular_velocity.z = this->get_imu_gyro_z() * gyro_scale_;
    imu_msg.angular_velocity_covariance[0] = gyro_covariance_;
    imu_msg.angular_velocity_covariance[4] = gyro_covariance_;
    imu_msg.angular_velocity_covariance[8] = gyro_covariance_;

    imu_msg.linear_acceleration.x = this->get_imu_acc_x() * accel_scale_;
    imu_msg.linear_acceleration.y = this->get_imu_acc_y() * accel_scale_;
    imu_msg.linear_acceleration.z = this->get_imu_acc_z() * accel_scale_;
    imu_msg.linear_acceleration_covariance[0] = accel_covariance_;
    imu_msg.linear_acceleration_covariance[4] = accel_covariance_;
    imu_msg.linear_acceleration_covariance[8] = accel_covariance_;

    imu_pub_->publish(imu_msg);

    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.frame_id = imu_frame_id_;
    mag_msg.header.stamp = now;

    mag_msg.magnetic_field.x = this->get_imu_mag_x() * mag_scale_;
    mag_msg.magnetic_field.y = this->get_imu_mag_y() * mag_scale_;
    mag_msg.magnetic_field.z = this->get_imu_mag_z() * mag_scale_;

    // TODO(fergs): mag covariance

    mag_pub_->publish(mag_msg);
  }
}

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::EtherbotixROS)
