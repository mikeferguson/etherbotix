/*
 * Copyright 2020-2023 Michael E. Ferguson
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

#include <math.h>
#include <string>

#include "angles/angles.h"
#include "etherbotix/etherbotix.hpp"
#include "etherbotix/dynamixel.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace etherbotix
{

/*
 * LD06 Packet structures
 */
typedef struct __attribute__((packed))
{
  uint16_t range;           // Range in millimeters
  uint8_t confidence;       // Around 200 for white objects within 6m
} ld06_measurement_t;

typedef struct __attribute__((packed))
{
  uint8_t start_byte;       // Always 0x54
  uint8_t length;           // Lower 5 bits are number of data measurements
  uint16_t radar_speed;     // Degrees per second - 10hz is 3600
  uint16_t start_angle;     // Angle in 0.01 degree increments, 0 is forward
  ld06_measurement_t data[12];
  uint16_t end_angle;       // Angle in 0.01 degree increments, 0 is forward
  uint16_t timestamp;       // In milliseconds
  uint8_t crc;
} ld06_packet_t;

class LD06Publisher : public rclcpp::Node, public Etherbotix
{
  static constexpr int BEAMS_PER_PACKET = 12;

public:
  explicit LD06Publisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ld06_publisher", options),
    Etherbotix(),
    initialized_(false),
    logger_(rclcpp::get_logger("ld06_publisher"))
  {
    // Declare parameters
    ip_ = this->declare_parameter<std::string>("ip_address", ip_);
    port_ = this->declare_parameter<int>("port", port_);
    RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);
    milliseconds_ = 10;

    scan_.header.frame_id = this->declare_parameter<std::string>("frame_id", "ld06_frame");
    scan_.range_min = 0.02;
    scan_.range_max = 12.0;

    // ROS2 interfaces
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

private:
  void publish()
  {
    RCLCPP_INFO(logger_, "Publish scan %f %f %lu",
                scan_.angle_min, scan_.angle_max, scan_.ranges.size());

    // Only publish if appropriately sized scan
    if (scan_.ranges.size() > 100)
    {
      // Finalize angle increment
      scan_.angle_increment = (scan_.angle_max - scan_.angle_min) / (scan_.ranges.size() - 1);

      // Publish the scan
      pub_->publish(scan_);
    }

    // Clean up scan for next go around
    scan_.ranges.clear();
    scan_.intensities.clear();
    scan_.angle_min = 0.0;
    scan_.angle_max = 0.0;
  }

  void update(const boost::system::error_code & e)
  {
    // Abort if we are shutting down
    if (!rclcpp::ok())
    {
      return;
    }

    // Grab current timestamp
    rclcpp::Time now = this->now();

    uint8_t buffer[256];

    if (!initialized_)
    {
      // Set usart baud to magical 255 - enables laser
      uint8_t len = insert_header(buffer);
      len += dynamixel::get_write_packet(
        &buffer[len],
        Etherbotix::ETHERBOTIX_ID,
        Etherbotix::REG_USART3_BAUD,
        {255});
      this->send(buffer, len);
      initialized_ = true;
    }

    uint8_t len = get(buffer);
    while (len > 0)
    {
      ld06_packet_t * packet = reinterpret_cast<ld06_packet_t*>(&buffer);

      // Verify packet
      if (packet->start_byte != 0x54)
      {
        scan_.ranges.clear();
        scan_.intensities.clear();
      }
      else if ((packet->length & 0x1f) != BEAMS_PER_PACKET)
      {
        scan_.ranges.clear();
        scan_.intensities.clear();
      }
      else
      {
        double time_per_point = 1 / 4500.0;
        double start_angle = angles::from_degrees(packet->start_angle * 0.01);
        double end_angle = angles::from_degrees(packet->end_angle * 0.01);
        double angle_increment = angles::shortest_angular_distance(start_angle, end_angle) /
                                 (BEAMS_PER_PACKET - 1);

        // Add data to scan message
        for (size_t i = 0; i < BEAMS_PER_PACKET; ++i)
        {
          // Compute heading of laser beam
          double angle = start_angle + (angle_increment * i);

          // Publish when we wrap around from 2pi to 0
          if (angle >= 2 * M_PI || angle < (scan_.angle_max - 0.05))
          {
            // Publish the completed scan
            publish();

            if (angle >= 2 * M_PI)
            {
              start_angle -= 2 * M_PI;
              angle = start_angle + (angle_increment * i);
            }
          }

          // Setup scan message header
          if (scan_.ranges.empty())
          {
            double offset = (BEAMS_PER_PACKET - i) * time_per_point;
            scan_.header.stamp = now - rclcpp::Duration::from_seconds(offset);
            scan_.angle_min = angle;
           }

          // Add the beam
          scan_.angle_max = angle;
          // Convert ranges from millimeters to meters
          scan_.ranges.push_back(packet->data[i].range * 0.001);
          // Publish raw intensity
          scan_.intensities.push_back(packet->data[i].confidence);
        }
      }

      len = get(buffer);
    }

    // Reset timer and async wait
    Etherbotix::update(e);
  }

private:
  bool initialized_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  sensor_msgs::msg::LaserScan scan_;

  // Book keeping for scan accumulation
  int packets_in_scan_;
};

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::LD06Publisher)