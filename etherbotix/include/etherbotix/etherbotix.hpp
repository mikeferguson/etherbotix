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

#ifndef ETHERBOTIX__ETHERBOTIX_HPP_
#define ETHERBOTIX__ETHERBOTIX_HPP_

#include <string>

#include "boost/array.hpp"
#include "boost/asio.hpp"

#include "rclcpp/rclcpp.hpp"

namespace etherbotix
{

class Etherbotix : public rclcpp::Node
{
public:
  explicit Etherbotix(const rclcpp::NodeOptions & options);
  virtual ~Etherbotix();

private:
  /** @brief Gets called at a periodic rate, updates everything. */
  void update(const boost::system::error_code & /*e*/);

  /** @brief We need to run the boost::asio::io_service in a thread. */
  void io_thread();

  /** @brief This sets up an async recieve from asio. */
  void start_receive();

  /** @brief Callback for asio receive. */
  void handle_receive(
    const boost::system::error_code & error,
    std::size_t bytes_transferred);

  boost::asio::io_service io_service_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  boost::array<char, 255> recv_buffer_;

  // 100hz timer for sending new data request
  boost::asio::deadline_timer timer_;

  // ROS2 interfaces
  rclcpp::Logger logger_;

  // ROS2 parameters
  std::string ip_;  // ip address to bind
  int port_;  // port to bind
  int64_t milliseconds_;  // duration between updates
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_HPP_
