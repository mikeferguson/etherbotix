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

#ifndef ETHERBOTIX__DYNAMIXEL_HPP_
#define ETHERBOTIX__DYNAMIXEL_HPP_

#include <vector>

namespace dynamixel
{

constexpr uint8_t BROADCAST_ID = 0xFE;

constexpr uint8_t READ_DATA = 2;
constexpr uint8_t WRITE_DATA = 3;
constexpr uint8_t SYNC_WRITE = 131;
constexpr uint8_t SYNC_READ = 132;

constexpr uint8_t PACKET_HEADER = 0;
constexpr uint8_t PACKET_HEADER_2 = 1;
constexpr uint8_t PACKET_ID = 2;
constexpr uint8_t PACKET_LEN = 3;
constexpr uint8_t PACKET_INSTR = 4;

constexpr double AX_SERVO_MAX_VEL = 11.93;  // 114 rpm
constexpr double AX_SERVO_RANGE = 5.23599;
constexpr double MX_SERVO_RANGE = 6.28319;

constexpr double GOAL_POSITION_L = 30;
constexpr double GOAL_POSITION_H = 31;
constexpr double PRESENT_POSITION_L = 36;
constexpr double PRESENT_POSITION_H = 37;

inline uint8_t compute_checksum(
  uint8_t* buffer,
  uint8_t length)
{
  uint8_t checksum = 0;
  for (uint8_t i = 2; i < length - 1; ++i)
  {
    checksum += buffer[i];
  }
  return (255 - checksum);
}

/**
 * @brief Get a READ_DATA packet.
 * @param buffer Buffer to fill with packet.
 * @param device_id The ID of the servo to read.
 * @param address Starting address of read.
 * @param length The number of bytes to read.
 */
inline uint8_t get_read_packet(
  uint8_t* buffer,
  uint8_t device_id,
  uint8_t address,
  uint8_t length)
{
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = device_id;
  buffer[3] = 4;  // Length of remaining packet
  buffer[4] = READ_DATA;
  buffer[5] = address;
  buffer[6] = length;
  buffer[7] = compute_checksum(buffer, 8);

  // Return number of bytes added to buffer
  return 8;
}

/**
 * @brief Get a SYNC_READ packet.
 * @param buffer Buffer to fill with packet.
 * @param device_ids Vector of servos to read.
 * @param address Starting address of read.
 * @param length The number of bytes to read.
 */
inline uint8_t get_sync_read_packet(
  uint8_t * buffer,
  std::vector<uint8_t> device_ids,
  uint8_t address,
  uint8_t length)
{
  uint8_t len = 0;
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  buffer[len++] = BROADCAST_ID;
  buffer[len++] = 4 + device_ids.size();  // Length of remaining packet
  buffer[len++] = SYNC_READ;
  buffer[len++] = address;
  buffer[len++] = length;
  for (auto id : device_ids)
  {
    buffer[len++] = id;
  }
  buffer[len++] = compute_checksum(buffer, 8 + device_ids.size());

  // Return number of bytes added to buffer
  return 8 + device_ids.size();
}

/**
 * @brief Get a WRITE_DATA packet.
 * @param buffer Buffer to fill with packet.
 * @param device_id The ID of the servo to read.
 * @param address Starting address of read.
 * @param length The number of bytes to read.
 */
inline uint8_t get_write_packet(
  uint8_t* buffer,
  uint8_t device_id,
  uint8_t address,
  std::vector<uint8_t> params)
{
  uint8_t len = 0;
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  buffer[len++] = device_id;
  buffer[len++] = 3 + params.size();  // Length of remaining packet
  buffer[len++] = WRITE_DATA;
  buffer[len++] = address;
  for (auto p : params)
  {
    buffer[len++] = p;
  }
  buffer[len++] = compute_checksum(buffer, 7 + params.size());

  // Return number of bytes added to buffer
  return len;
}

/**
 * @brief Create a SYNC_WRITE packet.
 * @param buffer Buffer to fill with packet.
 * @param address Starting address of write.
 * @params Each element of the vector should be:
 *         [id, param1, param2, ...]
 */
inline uint8_t get_sync_write_packet(
  uint8_t * buffer,
  uint8_t address,
  std::vector<std::vector<uint8_t>> params)
{
  if (params.empty() || params[0].empty())
  {
    return 0;
  }

  uint8_t param_len = params[0].size() * params.size();
  uint8_t len = 0;
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  buffer[len++] = BROADCAST_ID;
  buffer[len++] = 4 + param_len;  // remaining byte length
  buffer[len++] = SYNC_WRITE;
  buffer[len++] = address;
  buffer[len++] = params[0].size() - 1;  // how many byte to write each servo
  for (auto p : params)
  {
    for (auto x : p)
    {
      buffer[len++] = x;
    }
  }
  buffer[len++] = compute_checksum(buffer, 8 + param_len);

  // Return number of bytes added to buffer
  return 8 + param_len;
}

inline int get_baud_rate(uint8_t reg_value)
{
  if (reg_value == 1)
  {
    return 1000000;
  }
  else if (reg_value == 3)
  {
    return 500000;
  }
  else if (reg_value == 34)
  {
    return 57600;
  }
  else if (reg_value == 207)
  {
    return 9600;
  }
  else if (reg_value == 250)
  {
    return 2250000;
  }
  else if (reg_value == 251)
  {
    return 2500000;
  }
  else if (reg_value == 252)
  {
    return 3000000;
  }

  // Unknown baud
  return -1;
}

}  // namespace dynamixel

#endif  // ETHERBOTIX__DYNAMIXEL_HPP_
