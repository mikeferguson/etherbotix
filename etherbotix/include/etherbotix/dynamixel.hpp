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

#include "boost/array.hpp"

inline uint8_t compute_checksum(
  uint8_t* buffer,
  uint8_t length)
{
  uint8_t checksum = 0;
  for (uint8_t i = 2; i < length - 2; ++i)
  {
    checksum = buffer[i];
  }
  return 255 - checksum;
}

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
  buffer[4] = 2;  // AX_READ_DATA
  buffer[5] = address;
  buffer[6] = length;
  buffer[7] = compute_checksum(buffer, 8);

  // Return number of bytes added to buffer
  return 8;
}

#endif  // ETHERBOTIX__DYNAMIXEL_HPP_
