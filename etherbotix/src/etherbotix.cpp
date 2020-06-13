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

#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

using boost::asio::ip::udp;

namespace etherbotix
{

Etherbotix::Etherbotix(const rclcpp::NodeOptions & options)
: rclcpp::Node("etherbotix", options),
  socket_(io_service_),
  timer_(io_service_, boost::posix_time::milliseconds(10)),
  logger_(rclcpp::get_logger("etherbotix"))
{
  // Declare parameters
  ip_ = this->declare_parameter<std::string>("ip_address", "10.42.0.8");
  port_ = this->declare_parameter<int>("port", 5048);
  milliseconds_ = this->declare_parameter<int64_t>("update_interval_ms", 10);

  // Periodic update through a boost::asio timer
  timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));

  // any socket will do
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
  // kick the hardware to send some stuff back
  udp::endpoint receiver_endpoint =
    udp::endpoint(boost::asio::ip::address::from_string(ip_), port_);
  try {
    boost::array<char, 1> send_buf = {{0}};
    socket_.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }
  RCLCPP_INFO(logger_, "sent commands");

  // need to set a new expiration time before calling async_wait again
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(milliseconds_));
  timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));
}

void Etherbotix::io_thread()
{
  while (rclcpp::ok()) {
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
  if (!error || error == boost::asio::error::message_size) {
    recv_buffer_[bytes_transferred] = 0;  // tiny hack
    std::string data(recv_buffer_.begin(), recv_buffer_.end());
    RCLCPP_INFO(logger_, "%s", data.c_str());
    if (rclcpp::ok()) {
      start_receive();
    }
  }

  // ROS catches the ctrl-c, but we need to stop the io_service to exit
  if (!rclcpp::ok()) {
    socket_.get_io_service().stop();
  }
}

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::Etherbotix)
