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

#include "etherbotix/etherbotix.hpp"
#include "etherbotix/dynamixel.hpp"

#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

using boost::asio::ip::udp;

namespace etherbotix
{

Etherbotix::Etherbotix(const rclcpp::NodeOptions & options)
: rclcpp::Node("etherbotix", options),
  socket_(io_service_),
  update_timer_(io_service_, boost::posix_time::milliseconds(10)),
  logger_(rclcpp::get_logger("etherbotix"))
{
  // Declare parameters
  ip_ = this->declare_parameter<std::string>("ip_address", "192.168.0.42");
  port_ = this->declare_parameter<int>("port", 6707);
  milliseconds_ = this->declare_parameter<int64_t>("update_interval_ms", 10);
  RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);

  // Periodic update through a boost::asio timer
  update_timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));

  // Any socket will do
  socket_.open(udp::v4());
  start_receive();

  // Start a thread
  std::thread{boost::bind(&Etherbotix::io_thread, this)}.detach();
}

Etherbotix::~Etherbotix()
{
}

void Etherbotix::update(const boost::system::error_code & /*e*/)
{
  // Abort if we are shutting down
  if (!rclcpp::ok())
  {
    return;
  }

  // TODO: reset joint handle commands, set proper state

  // TODO: update controllers

  // Generate Commands
  size_t length = 0;
  boost::array<uint8_t, 4096> send_buf;

  send_buf[length++] = 0xff;
  send_buf[length++] = 'B';
  send_buf[length++] = 'O';
  send_buf[length++] = 'T';

  // Read 128 bytes from etherbotix
  length += get_read_packet(&send_buf[length], ETHERBOTIX_ID, 0, 128);

  // kick the hardware to send some stuff back
  udp::endpoint receiver_endpoint =
    udp::endpoint(boost::asio::ip::address::from_string(ip_), port_);
  try
  {
    socket_.send_to(boost::asio::buffer(send_buf, length), receiver_endpoint);
  }
  catch (std::exception & e)
  {
    std::cerr << e.what() << std::endl;
  }

  // need to set a new expiration time before calling async_wait again
  update_timer_.expires_at(update_timer_.expires_at() + boost::posix_time::milliseconds(milliseconds_));
  update_timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));
}

void Etherbotix::io_thread()
{
  while (rclcpp::ok())
  {
    io_service_.run();
  }
}

void Etherbotix::start_receive()
{
  socket_.async_receive_from(
    boost::asio::buffer(recv_buffer_), remote_endpoint_,
    boost::bind(&Etherbotix::handle_receive, this,
    boost::asio::placeholders::error,
    boost::asio::placeholders::bytes_transferred));
}

void Etherbotix::handle_receive(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  // This catches all the data, just print it for demo
  if (!error || error == boost::asio::error::message_size)
  {
    // Process response

    if (recv_buffer_[0] != 0xff ||
        recv_buffer_[1] != 'B' ||
        recv_buffer_[2] != 'O' ||
        recv_buffer_[3] != 'T')
    {
      RCLCPP_WARN(logger_, "Invalid MAGIC found!");
      bytes_transferred = 0;
    }

    size_t idx = 4;
    while (idx < bytes_transferred)
    {
      if (recv_buffer_[idx++] != 0xff ||
          recv_buffer_[idx++] != 0xff)
      {
        RCLCPP_WARN(logger_, "Invalid header found!");
        break;
      }

      uint8_t id = recv_buffer_[idx++];
      uint8_t len = recv_buffer_[idx++];
      if (idx + len + 2 > bytes_transferred)
      {
        RCLCPP_WARN(logger_, "Not enough bytes left!");
        break;
      }

      // TODO: checksum check

      uint8_t start_addr = recv_buffer_[idx++];
      for (uint8_t i = 0; i < len; ++i)
      {
        uint8_t addr = start_addr + i;
        if (addr == REG_VERSION)
        {
          version_ = recv_buffer_[idx + i];
        }
        // BAUD_RATE
        // DIGITAL_IN
        // DIGITAL_OUT
        // DIGITAL_DIR
        // USER_IO_USE
        // A0
        // A1
        // A2
        else if (addr == REG_SYSTEM_TIME)
        {
          system_time_ = (recv_buffer_[idx + i + 0] << 0) +
                         (recv_buffer_[idx + i + 1] << 8) +
                         (recv_buffer_[idx + i + 2] << 16) +
                         (recv_buffer_[idx + i + 3] << 24);
        }
        // SERVO CURRENT
        // AUX CURRENT
        // SYSTEM VOLTAGE
        // LED
        // IMU FLAGS
        // MOTOR_PERIOD
        // MOTOR_MAX_STEP
        // MOTOR1_VEL
        // MOTOR2_VEL
        // MOTOR1_POS
        // MOTOR2_POS
        // MOTOR1_CURRENT
        // MOTOR2_CURRENT
        // kp/kd/di/windup
        // imu
        // TIM12_MODE
        // TIM12_COUNT
        // PACKETS_RECV
        // PACKETS_BAD
      }

      //RCLCPP_INFO(logger_, "Got response from %d, with %d bytes of payload", id, len);
      RCLCPP_INFO(logger_, "%d: system time %d", id, system_time_);

      idx += len + 1;
    }

    if (rclcpp::ok())
    {
      start_receive();
    }
  }

  // ROS catches the ctrl-c, but we need to stop the io_service to exit
  if (!rclcpp::ok())
  {
    socket_.get_io_service().stop();
  }
}

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::Etherbotix)
