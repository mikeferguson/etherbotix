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
  /*
   * Register table definitions
   */

  // Standard Dynamixel Table
  static constexpr int REG_MODEL_NUMBER_L = 0;
  static constexpr int REG_MODEL_NUMBER_H = 1;
  static constexpr int REG_VERSION = 2;
  static constexpr int REG_ID = 3;
  static constexpr int REG_BAUD_RATE = 4;

  // Digital/Analog Access
  static constexpr int REG_DIGITAL_IN = 6;
  static constexpr int REG_DIGITAL_DIR = 7;
  static constexpr int REG_DIGITAL_OUT = 8;
  static constexpr int REG_USER_IO_USE = 9;
  static constexpr int REG_A0 = 10;
  static constexpr int REG_A1 = 12;
  static constexpr int REG_A2 = 14;

  // Other Data
  static constexpr int REG_SYSTEM_TIME = 16;
  static constexpr int REG_SERVO_CURRENT = 20;
  static constexpr int REG_AUX_CURRENT = 22;
  static constexpr int REG_SYSTEM_VOLTAGE = 24;
  static constexpr int REG_LED = 25;
  static constexpr int REG_IMU_FLAGS = 28;

  // Motors
  static constexpr int REG_MOTOR_PERIOD = 29;
  static constexpr int REG_MOTOR_MAX_STEP = 30;
  static constexpr int REG_MOTOR1_VEL = 32;
  static constexpr int REG_MOTOR2_VEL = 34;
  static constexpr int REG_MOTOR1_POS = 36;
  static constexpr int REG_MOTOR2_POS = 40;
  static constexpr int REG_MOTOR1_CURRENT = 44;
  static constexpr int REG_MOTOR2_CURRENT = 46;
  static constexpr int REG_MOTOR1_KP = 48;
  static constexpr int REG_MOTOR1_KD = 52;
  static constexpr int REG_MOTOR1_KI = 56;
  static constexpr int REG_MOTOR1_WINDUP = 60;
  static constexpr int REG_MOTOR2_KP = 64;
  static constexpr int REG_MOTOR2_KD = 68;
  static constexpr int REG_MOTOR2_KI = 72;
  static constexpr int REG_MOTOR2_WINDUP = 76;

  // IMU
  static constexpr int REG_ACC_X = 80;
  static constexpr int REG_ACC_Y = 82;
  static constexpr int REG_ACC_Z = 84;
  static constexpr int REG_GYRO_X = 86;
  static constexpr int REG_GYRO_Y = 88;
  static constexpr int REG_GYRO_Z = 90;
  static constexpr int REG_MAG_X = 92;
  static constexpr int REG_MAG_Y = 94;
  static constexpr int REG_MAG_Z = 96;

  // Device Setup
  static constexpr int REG_USART_BAUD = 98;
  static constexpr int REG_USART_CHAR = 99;
  static constexpr int REG_TIM9_MODE = 100;
  static constexpr int REG_TIM9_COUNT = 102;
  static constexpr int REG_TIM12_MODE = 104;
  static constexpr int REG_TIM12_COUNT = 106;
  static constexpr int REG_SPI_BAUD = 108;

  // Stats
  static constexpr int REG_PACKETS_RECV = 120;
  static constexpr int REG_PACKETS_BAD = 124;

  // Devices
  static constexpr int DEV_BOOTLOADER = 192;
  static constexpr int DEV_UNIQUE_ID = 193;
  static constexpr int DEV_M1_TRACE = 194;
  static constexpr int DEV_M2_TRACE = 195;

  // ID for the Etherbotix
  static constexpr int ETHERBOTIX_ID = 253;

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
  boost::array<uint8_t, 255> recv_buffer_;

  // 100hz timer for sending new data request
  boost::asio::deadline_timer update_timer_;

  // ROS2 interfaces
  rclcpp::Logger logger_;

  // ROS2 parameters
  std::string ip_;  // ip address to bind
  int port_;  // port to bind
  int64_t milliseconds_;  // duration between updates

  // Mirror of register table
  uint8_t version_;
  uint32_t system_time_;
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_HPP_
