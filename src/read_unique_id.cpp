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
#include <thread>

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
  len += dynamixel::get_read_packet(&buffer[len], Etherbotix::ETHERBOTIX_ID, Etherbotix::DEV_UNIQUE_ID, 12);

  Etherbotix e;
  while (e.get_unique_id() == "")
  {
    e.send(buffer, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << e.get_unique_id() << std::endl;

  return 0;
}
