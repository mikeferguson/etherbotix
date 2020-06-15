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

#include <iostream>

#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"

using etherbotix::Etherbotix;

int main (int argc, char *argv[])
{
  uint8_t buffer[256];
  int len = 0;
  buffer[len++] = 0xff;
  buffer[len++] = 'B';
  buffer[len++] = 'O';
  buffer[len++] = 'T';
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  buffer[len++] = Etherbotix::ETHERBOTIX_ID;
  buffer[len++] = 7;  // Length of remaining packet
  buffer[len++] = dynamixel::AX_WRITE_DATA;
  buffer[len++] = Etherbotix::DEV_BOOTLOADER;
  buffer[len++] = 'B';
  buffer[len++] = 'O';
  buffer[len++] = 'O';
  buffer[len++] = 'T';
  buffer[len++] = dynamixel::compute_checksum(&buffer[4], 11);

  Etherbotix e;
  e.send(buffer, len);

  return 0;
}
