/*
 * Copyright 2013-2024 Michael E. Ferguson
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

#ifndef ETHERBOTIX__ETHERBOTIX_ROS_HPP_
#define ETHERBOTIX__ETHERBOTIX_ROS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "etherbotix/etherbotix.hpp"
#include "etherbotix/dynamixel_servo.hpp"

#include "rclcpp/rclcpp.hpp"
#include "robot_controllers_interface/controller_manager.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace etherbotix
{

class EtherbotixROS : public rclcpp::Node, public Etherbotix
{
public:
  explicit EtherbotixROS(const rclcpp::NodeOptions & options);
  virtual ~EtherbotixROS();

protected:
  /** @brief Setup a motor from ROS params. */
  void setup_motor(const std::string & name,
                   const std::string & default_joint_name,
                   EtherbotixMotorPtr & motor);

  /** @brief Gets called at a periodic rate, updates everything. */
  virtual void update(const boost::system::error_code & /*e*/);

  /** @brief Publisher update callback. */
  void publish();

  /** @brief Diagnostics publisher callback. */
  void send_diagnostics();

  // Dynamixel Servos
  std::vector<DynamixelServoPtr> servos_;

  // ROS2 parameters
  std::string imu_frame_id_;
  double gyro_scale_;
  double gyro_covariance_;
  double accel_scale_;
  double accel_covariance_;
  double mag_scale_;

  // ROS2 interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  robot_controllers_interface::ControllerManagerPtr controller_manager_;
  bool initialized_;
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_ROS_HPP_
