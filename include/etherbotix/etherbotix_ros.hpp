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

#ifndef ETHERBOTIX__ETHERBOTIX_ROS_HPP_
#define ETHERBOTIX__ETHERBOTIX_ROS_HPP_

#include <memory>
#include <string>

#include "etherbotix/etherbotix.hpp"

#include "rclcpp/rclcpp.hpp"
#include "robot_controllers_interface/controller_manager.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace etherbotix
{

class EtherbotixROS : public rclcpp::Node, public Etherbotix
{
public:
  explicit EtherbotixROS(const rclcpp::NodeOptions & options);
  virtual ~EtherbotixROS();

protected:
  /** @brief Gets called at a periodic rate, updates everything. */
  virtual void update(const boost::system::error_code & /*e*/);

  /** @brief Publisher update callback. */
  void publish();

  // ROS2 interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  robot_controllers_interface::ControllerManagerPtr controller_manager_;
  bool initialized_;
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_ROS_HPP_
