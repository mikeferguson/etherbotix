/*
 * Copyright 2020 Michael E. Ferguson
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

#include <string>

#include "etherbotix/etherbotix.hpp"
#include "etherbotix/dynamixel.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rclcpp/rclcpp.hpp"

namespace etherbotix
{

class GpsPublisher : public rclcpp::Node, public Etherbotix
{
public:
  explicit GpsPublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("gps_publisher", options),
    Etherbotix(),
    initialized_(false),
    logger_(rclcpp::get_logger("gps_publisher"))
  {
    // Declare parameters
    ip_ = this->declare_parameter<std::string>("ip_address", ip_);
    port_ = this->declare_parameter<int>("port", port_);
    RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);

    milliseconds_ = this->declare_parameter<int>("update_interval_ms", 10);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");

    // ROS2 interfaces
    pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 10);
  }

private:
  void update(const boost::system::error_code & e)
  {
    // Abort if we are shutting down
    if (!rclcpp::ok())
    {
      return;
    }

    uint8_t buffer[256];

    if (!initialized_)
    {
      // Set baud to 9600, set terminating character to '\n'
      insert_header(buffer);
      buffer[4] = 0xff;
      buffer[5] = 0xff;
      buffer[6] = ETHERBOTIX_ID;
      buffer[7] = 5;  // Length of remaining packet
      buffer[8] = dynamixel::WRITE_DATA;
      buffer[9] = REG_USART3_BAUD;
      buffer[10] = 207;
      buffer[11] = '\n';
      buffer[12] = dynamixel::compute_checksum(&buffer[4], 9);
      this->send(buffer, 13);
      initialized_ = true;
    }

    uint8_t len = get(buffer);
    if (len > 0)
    {
      if (buffer[dynamixel::PACKET_ID] != ETHERBOTIX_ID)
      {
        RCLCPP_WARN(logger_, "Got packet for ID: %d", buffer[dynamixel::PACKET_ID]);
      }
      else
      {
        nmea_msgs::msg::Sentence msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = this->frame_id_;
        msg.sentence = std::string(reinterpret_cast<char*>(&buffer[5]), len - 6);

        pub_->publish(msg);
      }
    }

    // Reset timer and async wait
    Etherbotix::update(e);
  }

private:
  bool initialized_;
  std::string frame_id_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr pub_;
};

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::GpsPublisher)
