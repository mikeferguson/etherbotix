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
  static constexpr int BEAMS_PER_SCAN = 450;

  struct BeamData
  {
    float angle;
    float range;
    uint8_t intensity;
  };

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

    // Setup laser scan message
    scan_.header.frame_id = this->declare_parameter<std::string>("frame_id", "ld06_frame");
    resetScanMsg();

    // ROS2 interfaces
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

private:
  void resetScanMsg()
  {
    scan_.angle_min = 0.0;
    scan_.angle_max = 2 * M_PI;
    scan_.angle_increment = angles::from_degrees(360.0 / BEAMS_PER_SCAN);
    scan_.range_min = 0.005;
    scan_.range_max = 15.0;
    scan_.ranges.assign(BEAMS_PER_SCAN, std::numeric_limits<float>::quiet_NaN());
    scan_.intensities.assign(BEAMS_PER_SCAN, std::numeric_limits<float>::infinity());
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
      if (packet->start_byte == 0x54 &&
          (packet->length & 0x1f) == BEAMS_PER_PACKET)
      {
        double start_angle = angles::from_degrees(packet->start_angle * 0.01);
        double end_angle = angles::from_degrees(packet->end_angle * 0.01);
        double angle_increment = angles::shortest_angular_distance(start_angle, end_angle) /
                                 (BEAMS_PER_PACKET - 1);

        // Add data to beam vector
        for (size_t i = 0; i < BEAMS_PER_PACKET; ++i)
        {
          // Compute heading of laser beam
          double angle = start_angle + (angle_increment * i);

          // Publish when we wrap around from 2pi to 0
          if (angle >= 2 * M_PI || angle < 0.05)
          {
            // Publish the completed scan
            if (beams_.size() > 400)
            {
              RCLCPP_INFO(logger_, "Publish %lu beams", beams_.size());

              // Copy beams into scan
              for (auto beam : beams_)
              {
                // Data is mirrored in device
                size_t index = (scan_.angle_max - beam.angle) / scan_.angle_increment;
                if (index < scan_.ranges.size())
                {
                  if (beam.range < scan_.range_min)
                  {
                    scan_.ranges[index] = std::numeric_limits<float>::quiet_NaN();
                  }
                  else
                  {
                    scan_.ranges[index] = beam.range;
                  }
                  scan_.intensities[index] = beam.intensity;
                }
              }
              beams_.clear();

              // Publish the scan
              pub_->publish(scan_);

              // Clean up scan for next go around
              resetScanMsg();
            }

            if (angle >= 2 * M_PI)
            {
              start_angle -= 2 * M_PI;
              angle = start_angle + (angle_increment * i);
            }
          }

          // Setup scan message header
          if (beams_.empty())
          {
            double time_per_point = 1 / 4500.0;
            double offset = (BEAMS_PER_PACKET - i) * time_per_point;
            scan_.header.stamp = now - rclcpp::Duration::from_seconds(offset);
          }

          // Add the beam
          BeamData beam;
          beam.angle = angle;
          beam.range = packet->data[i].range * 0.001;
          beam.intensity = packet->data[i].confidence;
          beams_.push_back(beam);
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
  std::vector<BeamData> beams_;
};

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::LD06Publisher)
