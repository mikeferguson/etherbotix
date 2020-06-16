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

#include <boost/python.hpp>
#include <string>
#include <thread>

#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"

class EtherbotixWrapper : public etherbotix::Etherbotix
{
public:
  EtherbotixWrapper(const std::string & ip, int port)
  : Etherbotix(ip, port)
  {
  }

  // API to match old python drivers
  void write(uint8_t id, uint8_t start_addr, boost::python::list & values)
  {
    size_t len = boost::python::len(values);

    uint8_t buffer[256];
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = id;
    buffer[3] = len + 3;
    buffer[4] = dynamixel::AX_WRITE_DATA;
    buffer[5] = start_addr;
    for (size_t i = 0; i < len; ++i)
    {
      buffer[6 + i] = boost::python::extract<uint8_t>(values[i]);
    }
    buffer[6 + len] = dynamixel::compute_checksum(buffer, len + 7);
    Etherbotix::send(buffer, len + 7);
  }

  // API to match old python drivers
  boost::python::object read(uint8_t id, uint8_t start_addr, uint8_t length)
  {
    // Send read command
    uint8_t * buffer = new uint8_t[256];
    size_t len = dynamixel::get_read_packet(buffer, id, start_addr, length);
    this->send(buffer, len);

    // Read back packet
    do
    {
      len = this->get(buffer);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (len == 0);

    // Note: this will leak memory
    boost::python::object o(boost::python::handle<>(
      PyMemoryView_FromMemory(
        reinterpret_cast<char*>(buffer), len, PyBUF_READ)));
    return o;
  }
};

BOOST_PYTHON_MODULE(etherbotix_py)
{
  boost::python::class_<EtherbotixWrapper, boost::noncopyable>(
       "Etherbotix",
       boost::python::init<const std::string&, int>())
    .def("read", &EtherbotixWrapper::read)
    .def("write", &EtherbotixWrapper::write);

    // TODO(fergs): figure out how to wrap the constexpr
    // .def_readonly("DEV_M1_TRACE", &EtherbotixWrapper::DEV_M1_TRACE)
}
