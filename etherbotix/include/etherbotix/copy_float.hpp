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

#ifndef ETHERBOTIX__COPY_FLOAT_HPP_
#define ETHERBOTIX__COPY_FLOAT_HPP_

namespace etherbotix
{

/**
 *  @brief Copy a float into possibly unaligned
 *         data structure.
 *  @param f The float to copy into data structure.
 *  @param t The data structure to copy float into.
 */
template<typename T>
void copy_float(float f, T& t)
{
  uint8_t* f_as_uint8 = reinterpret_cast<uint8_t*>(&f);
  uint8_t* t_as_uint8 = reinterpret_cast<uint8_t*>(&t);

  for (std::size_t i = 0; i < 4; ++i)
  {
    t_as_uint8[i] = f_as_uint8[i];
  }
}

/**
 *  @brief Get float from possibly unaligned
 *         data structure.
 *  @param t The data structure to read as float.
 *  @returns The float value.
 */
template<typename T>
float copy_float(T& t)
{
  float f;

  uint8_t* f_as_uint8 = reinterpret_cast<uint8_t*>(&f);
  uint8_t* t_as_uint8 = reinterpret_cast<uint8_t*>(&t);

  for (std::size_t i = 0; i < 4; ++i)
  {
    f_as_uint8[i] = t_as_uint8[i];
  }

  return f;
}

}  // namespace etherbotix

#endif  // ETHERBOTIX__COPY_FLOAT_HPP_
