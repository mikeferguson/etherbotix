/*
 * Copyright 2020-2024 Michael E. Ferguson
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
    buffer[4] = dynamixel::WRITE_DATA;
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

  // All-in-one to update the state
  int update()
  {
    int packets_sent = 0;
    uint8_t buffer[256];
    int len = 0;
    len = etherbotix::insert_header(buffer);
    // Read full table
    len += dynamixel::get_read_packet(&buffer[len], ETHERBOTIX_ID, 0, 128);
    ++packets_sent;
    // Make sure unique ID is filled in
    if (this->get_unique_id().empty())
    {
      len += dynamixel::get_read_packet(&buffer[len], ETHERBOTIX_ID,
                                        DEV_UNIQUE_ID, 12);
      ++packets_sent;
    }
    len += m1_->get_packets(&buffer[len], 1);
    len += m2_->get_packets(&buffer[len], 2);
    this->send(buffer, len);
    return packets_sent;
  }
};

BOOST_PYTHON_MODULE(etherbotix_py)
{
  boost::python::class_<EtherbotixWrapper, boost::noncopyable>(
       "Etherbotix",
       boost::python::init<const std::string&, int>())
    .def("get_version", &EtherbotixWrapper::get_version)
    .def("get_baud_rate", &EtherbotixWrapper::get_baud_rate)
    .def("get_digital_in", &EtherbotixWrapper::get_digital_in)
    .def("get_digital_out", &EtherbotixWrapper::get_digital_out)
    .def("get_digital_dir", &EtherbotixWrapper::get_digital_dir)
    .def("get_user_io_use", &EtherbotixWrapper::get_user_io_use)
    .def("get_analog0", &EtherbotixWrapper::get_analog0)
    .def("get_analog1", &EtherbotixWrapper::get_analog1)
    .def("get_analog2", &EtherbotixWrapper::get_analog2)
    .def("get_system_time", &EtherbotixWrapper::get_system_time)
    .def("get_servo_current", &EtherbotixWrapper::get_servo_current)
    .def("get_aux_current", &EtherbotixWrapper::get_aux_current)
    .def("get_system_voltage", &EtherbotixWrapper::get_system_voltage)
    .def("get_motor_period", &EtherbotixWrapper::get_motor_period)
    .def("get_motor_max_step", &EtherbotixWrapper::get_motor_max_step)
    .def("get_motor1", &EtherbotixWrapper::getMotor1)
    .def("get_motor2", &EtherbotixWrapper::getMotor2)
    .def("get_imu_version", &EtherbotixWrapper::get_imu_version)
    .def("get_imu_flags", &EtherbotixWrapper::get_imu_flags)
    .def("get_imu_acc_x", &EtherbotixWrapper::get_imu_acc_x)
    .def("get_imu_acc_y", &EtherbotixWrapper::get_imu_acc_y)
    .def("get_imu_acc_z", &EtherbotixWrapper::get_imu_acc_z)
    .def("get_imu_gyro_x", &EtherbotixWrapper::get_imu_gyro_x)
    .def("get_imu_gyro_y", &EtherbotixWrapper::get_imu_gyro_y)
    .def("get_imu_gyro_z", &EtherbotixWrapper::get_imu_gyro_z)
    .def("get_imu_mag_x", &EtherbotixWrapper::get_imu_mag_x)
    .def("get_imu_mag_y", &EtherbotixWrapper::get_imu_mag_y)
    .def("get_imu_mag_z", &EtherbotixWrapper::get_imu_mag_z)
    .def("get_usart3_baud", &EtherbotixWrapper::get_usart3_baud)
    .def("get_usart3_char", &EtherbotixWrapper::get_usart3_char)
    .def("get_tim12_mode", &EtherbotixWrapper::get_tim12_mode)
    .def("get_tim12_count", &EtherbotixWrapper::get_tim12_count)
    .def("get_packets_recv", &EtherbotixWrapper::get_packets_recv)
    .def("get_packets_bad", &EtherbotixWrapper::get_packets_bad)
    .def("get_unique_id", &EtherbotixWrapper::get_unique_id)
    .def("set_digital_pin", &EtherbotixWrapper::set_digital_pin)
    .def("read", &EtherbotixWrapper::read)
    .def("write", &EtherbotixWrapper::write)
    .def("update", &EtherbotixWrapper::update);

  boost::python::class_<etherbotix::EtherbotixMotor, boost::noncopyable>(
       "EtherbotixMotor",
       boost::python::init<const std::string&, double>())
    .def("get_name", &etherbotix::EtherbotixMotor::getName)
    .def("get_position", &etherbotix::EtherbotixMotor::getPosition)
    .def("get_velocity", &etherbotix::EtherbotixMotor::getVelocity)
    .def("get_effort", &etherbotix::EtherbotixMotor::getEffort)
    .def("set_velocity", &etherbotix::EtherbotixMotor::setVelocity)
    .def("set_position", &etherbotix::EtherbotixMotor::setPosition)
    .def("set_ticks_per_radian", &etherbotix::EtherbotixMotor::set_ticks_per_radian)
    .def("set_ticks_offset", &etherbotix::EtherbotixMotor::set_ticks_offset);

  boost::python::register_ptr_to_python<std::shared_ptr<etherbotix::EtherbotixMotor>>();

    // TODO(fergs): figure out how to wrap the constexpr
    // .def_readonly("DEV_M1_TRACE", &EtherbotixWrapper::DEV_M1_TRACE)
}
