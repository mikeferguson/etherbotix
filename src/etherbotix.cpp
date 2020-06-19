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

#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "etherbotix/etherbotix.hpp"
#include "etherbotix/copy_float.hpp"
#include "etherbotix/dynamixel.hpp"

#include "boost/bind.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

using boost::asio::ip::udp;

namespace etherbotix
{

uint8_t insert_header(uint8_t * buffer)
{
  buffer[0] = 0xff;
  buffer[1] = 'B';
  buffer[2] = 'O';
  buffer[3] = 'T';
  return 4;
}

Etherbotix::Etherbotix(const std::string & ip, int port, int millisecond)
: socket_(io_service_),
  update_timer_(io_service_, boost::posix_time::milliseconds(millisecond)),
  ip_(ip),
  port_(port),
  milliseconds_(millisecond),
  version_(-1),
  baud_rate_(-1),
  digital_in_(-1),
  digital_out_(-1),
  digital_dir_(-1),
  user_io_use_(-1),
  analog0_(-1),
  analog1_(-1),
  analog2_(-1),
  system_time_(0),
  servo_current_(0.0),
  aux_current_(0.0),
  system_voltage_(0.0),
  imu_flags_(-1),
  motor_period_(-1),
  motor_max_step_(-1),
  imu_acc_x_(0),
  imu_acc_y_(0),
  imu_acc_z_(0),
  imu_gyro_x_(0),
  imu_gyro_y_(0),
  imu_gyro_z_(0),
  imu_mag_x_(0),
  imu_mag_y_(0),
  imu_mag_z_(0),
  usart3_baud_(-1),
  usart3_char_(0),
  tim9_mode_(-1),
  tim9_count_(0),
  tim12_mode_(-1),
  tim12_count_(0),
  spi2_baud_(-1),
  packets_recv_(0),
  packets_bad_(0),
  unique_id_("")
{
  // Setup motors
  left_motor_ = std::make_shared<EtherbotixMotor>("l_wheel_joint", 1);
  right_motor_ = std::make_shared<EtherbotixMotor>("r_wheel_joint", 1);

  // Periodic update through a boost::asio timer
  update_timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));

  // Any socket will do
  socket_.open(udp::v4());
  start_receive();

  // Start a thread
  std::thread{std::bind(&Etherbotix::io_thread, this)}.detach();
}

Etherbotix::~Etherbotix()
{
  shutdown();
}

bool Etherbotix::set_digital_pin(uint8_t index, uint8_t value, uint8_t direction)
{
  if (index > 7)
  {
    return false;
  }

  if (digital_in_ < 0 || digital_out_ < 0 || digital_dir_ < 0)
  {
    // We can't update values unless we know the current value
    return false;
  }

  uint8_t mask = std::pow(2, index);
  if (value > 0)
  {
    value = mask;
  }

  if (direction > 0)
  {
    direction = mask;
  }
  value = (digital_out_ & ~mask) | value;
  direction = (digital_dir_ & ~mask) | direction;

  // Copy locally in case we make multiple changes before read
  digital_out_ = value;
  digital_dir_ = direction;

  uint8_t buffer[64];
  uint8_t len = dynamixel::get_write_packet(
    buffer,
    ETHERBOTIX_ID,
    REG_DIGITAL_DIR,
    {direction, value});
  send(buffer, len);

  return true;
}

bool Etherbotix::set_tim12_mode(uint8_t mode)
{
  uint8_t buffer[64];
  uint8_t len = dynamixel::get_write_packet(
    buffer,
    ETHERBOTIX_ID,
    REG_TIM12_MODE,
    {mode});
  send(buffer, len);
  return true;
}

void Etherbotix::send(const uint8_t * buffer, size_t len)
{
  // Send packets to hardware
  udp::endpoint receiver_endpoint =
    udp::endpoint(boost::asio::ip::address::from_string(ip_), port_);
  try
  {
    if (buffer[1] == 'B' && buffer[2] == 'O' && buffer[3] == 'T' && buffer[0] == 0xff)
    {
      // Send without copy
      socket_.send_to(boost::asio::buffer(buffer, len), receiver_endpoint);
      return;
    }

    // Need to copy and add header
    uint8_t b[256];
    uint8_t b_len = insert_header(b);
    for (size_t i = 0; i <  len; ++i)
    {
      b[b_len++] = buffer[i];
    }
    socket_.send_to(boost::asio::buffer(b, b_len), receiver_endpoint);
  }
  catch (std::exception & e)
  {
    std::cerr << e.what() << std::endl;
  }
}

uint8_t Etherbotix::get(uint8_t * buffer)
{
  std::lock_guard<std::mutex> lock(packets_mutex_);
  if (!packets_.empty())
  {
    uint8_t len = packets_.front().size;
    for (uint8_t i = 0; i < len; ++i)
    {
      buffer[i] = packets_.front().data[i];
    }
    packets_.pop();
    return len;
  }
  return 0;
}

void Etherbotix::update(const boost::system::error_code & /*e*/)
{
  // Need to set a new expiration time before calling async_wait again
  update_timer_.expires_at(update_timer_.expires_at() +
                           boost::posix_time::milliseconds(milliseconds_));
  update_timer_.async_wait(boost::bind(&Etherbotix::update, this, _1));
}

void Etherbotix::io_thread()
{
  io_service_.run();
}

void Etherbotix::shutdown()
{
  io_service_.stop();
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
  if (!error || error == boost::asio::error::message_size)
  {
    // Process response
    if (recv_buffer_[0] != 0xff ||
        recv_buffer_[1] != 'B' ||
        recv_buffer_[2] != 'O' ||
        recv_buffer_[3] != 'T')
    {
      // Invalid ethernet header
      bytes_transferred = 0;
    }

    size_t idx = 4;
    while (idx < bytes_transferred)
    {
      size_t start = idx;

      if (recv_buffer_[idx++] != 0xff ||
          recv_buffer_[idx++] != 0xff)
      {
        // Invalid packet header
        break;
      }

      uint8_t id = recv_buffer_[idx++];
      uint8_t len = recv_buffer_[idx++];
      if (idx + len > bytes_transferred)
      {
        // Not enough bytes left
        break;
      }

      // TODO(fergs): checksum check

      // Set length to just the parameters
      len -= 2;

      // Handle packets from servos, etc
      if (id != ETHERBOTIX_ID)
      {
        std::lock_guard<std::mutex> lock(packets_mutex_);
        packets_.emplace(Packet(len + 6, &recv_buffer_[start]));
        idx += len + 1;
        continue;
      }

      uint8_t start_addr = recv_buffer_[idx++];
      if (start_addr >= 128)
      {
        // Etherbotix Device
        if (start_addr == DEV_UNIQUE_ID)
        {
          if (len == 12)
          {
            std::stringstream uid;
            for (size_t j = 0; j < 12; j++)
            {
              uid << std::hex << std::uppercase <<
                     std::setw(2) << std::setfill('0') <<
                     static_cast<int>(recv_buffer_[idx + j]);
            }
            unique_id_ = uid.str();
          }
        }

        // Not a valid device, but no need to process it as regular table below
        std::lock_guard<std::mutex> lock(packets_mutex_);
        packets_.emplace(Packet(len + 6, &recv_buffer_[start]));
        idx += len + 1;
        continue;
      }

      // Etherbotix register table
      for (uint8_t i = 0; i < len; ++i)
      {
        uint8_t addr = start_addr + i;
        if (addr == REG_VERSION)
        {
          version_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_BAUD_RATE)
        {
          uint8_t baud = recv_buffer_[idx + i];
          baud_rate_ = dynamixel::get_baud_rate(baud);
        }
        else if (addr == REG_DIGITAL_IN)
        {
          digital_in_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_DIGITAL_OUT)
        {
          digital_out_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_DIGITAL_DIR)
        {
          digital_dir_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_USER_IO_USE)
        {
          user_io_use_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_A0)
        {
          uint16_t value = (recv_buffer_[idx + i + 0] << 0) +
                           (recv_buffer_[idx + i + 1] << 8);
          analog0_ = value;
        }
        else if (addr == REG_A1)
        {
          uint16_t value = (recv_buffer_[idx + i + 0] << 0) +
                           (recv_buffer_[idx + i + 1] << 8);
          analog1_ = value;
        }
        else if (addr == REG_A2)
        {
          uint16_t value = (recv_buffer_[idx + i + 0] << 0) +
                           (recv_buffer_[idx + i + 1] << 8);
          analog2_ = value;
        }
        else if (addr == REG_SYSTEM_TIME)
        {
          system_time_ = (recv_buffer_[idx + i + 0] << 0) +
                         (recv_buffer_[idx + i + 1] << 8) +
                         (recv_buffer_[idx + i + 2] << 16) +
                         (recv_buffer_[idx + i + 3] << 24);
        }
        else if (addr == REG_SERVO_CURRENT)
        {
          int16_t current = (recv_buffer_[idx + i + 0] << 0) +
                            (recv_buffer_[idx + i + 1] << 8);
          servo_current_ = current / 1000.0;  // mA - > A
        }
        else if (addr == REG_AUX_CURRENT)
        {
          int16_t current = (recv_buffer_[idx + i + 0] << 0) +
                            (recv_buffer_[idx + i + 1] << 8);
          aux_current_ = current / 1000.0;  // mA -> A
        }
        else if (addr == REG_SYSTEM_VOLTAGE)
        {
          system_voltage_ = static_cast<float>(recv_buffer_[idx + i]) / 10.0;
        }
        // LED
        else if (addr == REG_IMU_FLAGS)
        {
          imu_flags_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_MOTOR_PERIOD)
        {
          motor_period_ = recv_buffer_[idx + i];
          left_motor_->update_motor_period_from_packet(motor_period_);
          right_motor_->update_motor_period_from_packet(motor_period_);
        }
        else if (addr == REG_MOTOR_MAX_STEP)
        {
          motor_max_step_ = (recv_buffer_[idx + i + 0] << 0) +
                            (recv_buffer_[idx + i + 1] << 8);
        }
        else if (addr == REG_MOTOR1_VEL)
        {
          // TODO(fergs): verify length remaining
          int16_t vel = (recv_buffer_[idx + i + 0] << 0) +
                        (recv_buffer_[idx + i + 1] << 8);
          int32_t pos = (recv_buffer_[idx + i + 4] << 0) +
                        (recv_buffer_[idx + i + 5] << 8) +
                        (recv_buffer_[idx + i + 6] << 16) +
                        (recv_buffer_[idx + i + 7] << 24);
          int16_t cur = (recv_buffer_[idx + i + 12] << 0) +
                        (recv_buffer_[idx + i + 13] << 8);
          left_motor_->update_from_packet(vel, pos, cur);
        }
        else if (addr == REG_MOTOR2_VEL)
        {
          // TODO(fergs): verify length remaining
          int16_t vel = (recv_buffer_[idx + i + 0] << 0) +
                        (recv_buffer_[idx + i + 1] << 8);
          int32_t pos = (recv_buffer_[idx + i + 6] << 0) +
                        (recv_buffer_[idx + i + 7] << 8) +
                        (recv_buffer_[idx + i + 8] << 16) +
                        (recv_buffer_[idx + i + 9] << 24);
          int16_t cur = (recv_buffer_[idx + i + 12] << 0) +
                        (recv_buffer_[idx + i + 13] << 8);
          right_motor_->update_from_packet(vel, pos, cur);
        }
        else if (addr == REG_MOTOR1_KP)
        {
          float kp = copy_float(recv_buffer_[idx + i + 0]);
          float kd = copy_float(recv_buffer_[idx + i + 4]);
          float ki = copy_float(recv_buffer_[idx + i + 8]);
          float windup = copy_float(recv_buffer_[idx + i + 12]);
          left_motor_->update_gains_from_packet(kp, kd, ki, windup);
        }
        else if (addr == REG_MOTOR2_KP)
        {
          float kp = copy_float(recv_buffer_[idx + i + 0]);
          float kd = copy_float(recv_buffer_[idx + i + 4]);
          float ki = copy_float(recv_buffer_[idx + i + 8]);
          float windup = copy_float(recv_buffer_[idx + i + 12]);
          right_motor_->update_gains_from_packet(kp, kd, ki, windup);
        }
        else if (addr == REG_ACC_X)
        {
          // TODO(fergs): verify length remaining
          imu_acc_x_ = (recv_buffer_[idx + i + 0] << 0) +
                       (recv_buffer_[idx + i + 1] << 8);
          imu_acc_y_ = (recv_buffer_[idx + i + 2] << 0) +
                       (recv_buffer_[idx + i + 3] << 8);
          imu_acc_z_ = (recv_buffer_[idx + i + 4] << 0) +
                       (recv_buffer_[idx + i + 5] << 8);
          imu_gyro_x_ = (recv_buffer_[idx + i + 6] << 0) +
                        (recv_buffer_[idx + i + 7] << 8);
          imu_gyro_y_ = (recv_buffer_[idx + i + 8] << 0) +
                        (recv_buffer_[idx + i + 9] << 8);
          imu_gyro_z_ = (recv_buffer_[idx + i + 10] << 0) +
                        (recv_buffer_[idx + i + 11] << 8);
          imu_mag_x_ = (recv_buffer_[idx + i + 12] << 0) +
                       (recv_buffer_[idx + i + 13] << 8);
          imu_mag_y_ = (recv_buffer_[idx + i + 14] << 0) +
                       (recv_buffer_[idx + i + 15] << 8);
          imu_mag_z_ = (recv_buffer_[idx + i + 16] << 0) +
                       (recv_buffer_[idx + i + 17] << 8);
        }
        else if (addr == REG_USART3_BAUD)
        {
          uint8_t baud = recv_buffer_[idx + i];
          usart3_baud_ = dynamixel::get_baud_rate(baud);
        }
        else if (addr == REG_USART3_CHAR)
        {
          usart3_char_ = recv_buffer_[idx + i];
        }
        else if (addr == REG_TIM9_MODE)
        {
          tim9_mode_ = (recv_buffer_[idx + i + 0] << 0) +
                       (recv_buffer_[idx + i + 1] << 8);
        }
        else if (addr == REG_TIM9_COUNT)
        {
          tim9_count_ = (recv_buffer_[idx + i + 0] << 0) +
                        (recv_buffer_[idx + i + 1] << 8);
        }
        else if (addr == REG_TIM12_MODE)
        {
          tim12_mode_ = (recv_buffer_[idx + i + 0] << 0) +
                        (recv_buffer_[idx + i + 1] << 8);
        }
        else if (addr == REG_TIM12_COUNT)
        {
          tim12_count_ = (recv_buffer_[idx + i + 0] << 0) +
                         (recv_buffer_[idx + i + 1] << 8);
        }
        else if (addr == REG_SPI2_BAUD)
        {
          uint8_t baud = recv_buffer_[idx + i];
          spi2_baud_ = dynamixel::get_baud_rate(baud);
        }
        else if (addr == REG_PACKETS_RECV)
        {
          packets_recv_ = (recv_buffer_[idx + i + 0] << 0) +
                          (recv_buffer_[idx + i + 1] << 8) +
                          (recv_buffer_[idx + i + 2] << 16) +
                          (recv_buffer_[idx + i + 3] << 24);
        }
        else if (addr == REG_PACKETS_BAD)
        {
          packets_bad_ = (recv_buffer_[idx + i + 0] << 0) +
                         (recv_buffer_[idx + i + 1] << 8) +
                         (recv_buffer_[idx + i + 2] << 16) +
                         (recv_buffer_[idx + i + 3] << 24);
        }
      }

      idx += len + 1;
    }

    start_receive();
  }
}

}  // namespace etherbotix
