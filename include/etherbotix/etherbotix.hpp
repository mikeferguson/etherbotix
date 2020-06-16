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

#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "boost/array.hpp"
#include "boost/asio.hpp"

#include "etherbotix/etherbotix_motor.hpp"

namespace etherbotix
{

/** @brief Put the ethernet header in a buffer. */
uint8_t insert_header(uint8_t * buffer);

class Etherbotix
{
  struct Packet
  {
    Packet(uint8_t len, const uint8_t * buffer)
    {
      size = len;
      for (size_t i = 0; i < len; ++i)
      {
        data[i] = buffer[i];
      }
    }
    uint8_t size;
    boost::array<uint8_t, 256> data;
  };

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
  static constexpr int REG_USART3_BAUD = 98;
  static constexpr int REG_USART3_CHAR = 99;
  static constexpr int REG_TIM9_MODE = 100;
  static constexpr int REG_TIM9_COUNT = 102;
  static constexpr int REG_TIM12_MODE = 104;
  static constexpr int REG_TIM12_COUNT = 106;
  static constexpr int REG_SPI2_BAUD = 108;

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

  explicit Etherbotix(const std::string & ip = "192.168.0.42",
                      int port = 6707,
                      int millisecond = 10);
  virtual ~Etherbotix();

  /** @brief Get the version of this board, -1 if not yet read. */
  int get_version() { return version_; }

  /** @brief Get the baud rate for the Dynamixel bus. -1 if not read. */
  int get_baud_rate() { return baud_rate_; }

  int get_digital_in() { return digital_in_; }
  int get_digital_out() { return digital_out_; }
  int get_digital_dir() { return digital_dir_; }
  int get_user_io_use() { return user_io_use_; }

  // TODO(fergs): set digital/analog/etc

  /** @brief Get the board system time in milliseconds. */
  uint32_t get_system_time() { return system_time_; }

  /** @brief Get the servo current in amperes. */
  float get_servo_current() { return servo_current_; }

  /** @brief Get the aux current in amperes. */
  float get_aux_current() { return aux_current_; }

  /** @brief Get the system voltage in volts. */
  float get_system_voltage() { return system_voltage_; }

  /** @brief Get the time (in milliseconds) between motor PID updates. */
  int get_motor_period() { return motor_period_; }

  int get_motor_max_step() { return motor_max_step_; }

  EtherbotixMotorPtr getLeftMotor() { return left_motor_; }
  EtherbotixMotorPtr getRightMotor() { return right_motor_; }

  int get_imu_version()
  {
    if (imu_flags_ < 0)
    {
      return imu_flags_;
    }
    return (imu_flags_ & 0x0f);
  }
  int get_imu_acc_x() { return imu_acc_x_; }
  int get_imu_acc_y() { return imu_acc_y_; }
  int get_imu_acc_z() { return imu_acc_z_; }
  int get_imu_gyro_x() { return imu_gyro_x_; }
  int get_imu_gyro_y() { return imu_gyro_y_; }
  int get_imu_gyro_z() { return imu_gyro_z_; }
  int get_imu_mag_x() { return imu_mag_x_; }
  int get_imu_mag_y() { return imu_mag_y_; }
  int get_imu_mag_z() { return imu_mag_z_; }

  int get_usart3_baud() { return usart3_baud_; }
  int get_usart3_char() { return usart3_char_; }

  int get_tim12_mode() { return tim12_mode_; }
  int get_tim12_count() { return tim12_count_; }

  uint32_t get_packets_recv() { return packets_recv_; }
  uint32_t get_packets_bad() { return packets_bad_; }

  std::string get_unique_id() { return unique_id_; }

  /** @brief Send a packet to Etherbotix. */
  void send(const uint8_t * buffer, size_t len);

  /** @brief Get a packet into buffer. */
  uint8_t get(uint8_t * buffer);

protected:
  /** @brief Gets called at a periodic rate, updates everything. */
  virtual void update(const boost::system::error_code & /*e*/);

  /** @brief We need to run the boost::asio::io_service in a thread. */
  void io_thread();

  /** @brief Shutdown the ASIO services. */
  void shutdown();

  /** @brief This sets up an async recieve from asio. */
  void start_receive();

  /** @brief Callback for asio receive. */
  void handle_receive(
    const boost::system::error_code & error,
    std::size_t bytes_transferred);

  boost::asio::io_service io_service_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  boost::array<uint8_t, 256> recv_buffer_;

  // Packets from devices
  std::mutex packets_mutex_;
  std::queue<Packet> packets_;

  // 100hz timer for sending new data request
  boost::asio::deadline_timer update_timer_;

  // Ethernet interface
  std::string ip_;
  int port_;

  // Duration between updates
  int64_t milliseconds_;

  // Mirror of register table
  int version_;
  int baud_rate_;  // dynamixel bus baud in bps
  int digital_in_;
  int digital_out_;
  int digital_dir_;
  int user_io_use_;  // mask over digital IO
  int analog0_;
  int analog1_;
  int analog2_;
  uint32_t system_time_;  // milliseconds
  float servo_current_;  // amsp
  float aux_current_;  // amps
  float system_voltage_;  // volts
  int imu_flags_;
  int motor_period_;
  int motor_max_step_;
  EtherbotixMotorPtr left_motor_;
  EtherbotixMotorPtr right_motor_;
  int16_t imu_acc_x_;
  int16_t imu_acc_y_;
  int16_t imu_acc_z_;
  int16_t imu_gyro_x_;
  int16_t imu_gyro_y_;
  int16_t imu_gyro_z_;
  int16_t imu_mag_x_;
  int16_t imu_mag_y_;
  int16_t imu_mag_z_;
  int usart3_baud_;  // baud rate in bps
  uint8_t usart3_char_;  // temrminating character
  int tim9_mode_;
  int tim9_count_;
  int tim12_mode_;
  int tim12_count_;
  int spi2_baud_;  // baud rate in bps
  uint32_t packets_recv_;
  uint32_t packets_bad_;
  std::string unique_id_;
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_HPP_
