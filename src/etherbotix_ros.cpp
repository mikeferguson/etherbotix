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
  milliseconds_ = this->declare_parameter<int>("update_interval_ms", 10);
  RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);

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

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();

  msg.name.push_back(left_motor_->getName());
  msg.position.push_back(left_motor_->getPosition());
  msg.velocity.push_back(left_motor_->getVelocity());
  msg.effort.push_back(left_motor_->getEffort());

  msg.name.push_back(right_motor_->getName());
  msg.position.push_back(right_motor_->getPosition());
  msg.velocity.push_back(right_motor_->getVelocity());
  msg.effort.push_back(right_motor_->getEffort());

  joint_state_pub_->publish(msg);
}

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::EtherbotixROS)
