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

int main ()
{
  uint8_t buffer[256];
  int len = 0;
  len = etherbotix::insert_header(buffer);
  // Read full table (128 bytes)
  len += dynamixel::get_read_packet(&buffer[len], Etherbotix::ETHERBOTIX_ID, 0, 128);
  len += dynamixel::get_read_packet(&buffer[len], Etherbotix::ETHERBOTIX_ID,
                                    Etherbotix::DEV_UNIQUE_ID, 12);

  Etherbotix e;
  while (e.get_system_time() == 0 || e.get_unique_id() == "")
  {
    e.send(buffer, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "System Time:    " << e.get_system_time() << std::endl;
  std::cout << "Version:        " << e.get_version() << std::endl;
  std::cout << "Unique ID:      " << e.get_unique_id() << std::endl;
  std::cout << "Packets Recv:   " << e.get_packets_recv() << std::endl;
  std::cout << "Packets Bad:    " << e.get_packets_bad() << std::endl;
  std::cout << "Baud Rate:      " << e.get_baud_rate() << std::endl;
  std::cout << "Digital In:     " << e.get_digital_in() << std::endl;
  std::cout << "Digital Out:    " << e.get_digital_out() << std::endl;
  std::cout << "Digital Dir:    " << e.get_digital_dir() << std::endl;
  std::cout << "System Voltage: " << e.get_system_voltage() << " V" << std::endl;
  std::cout << "Servo Current:  " << e.get_servo_current() << " A" << std::endl;
  std::cout << "Aux Current:    " << e.get_aux_current() << " A" << std::endl;
  std::cout << "Motor 1" << std::endl;
  std::cout << "  Velocity:     " << e.getLeftMotor()->getVelocity() << std::endl;
  std::cout << "  Position:     " << e.getLeftMotor()->getPosition() << std::endl;
  std::cout << "  Current:      " << e.getLeftMotor()->getEffort() << " A" << std::endl;
  float kp, kd, ki, windup;
  if (e.getLeftMotor()->get_gains(kp, kd, ki, windup))
  {
    std::cout << "  Gains:        Kp: " << kp << ", Kd: " << kd <<
                 ", Ki: " << ki << ", Windup: " << windup << std::endl;
  }
  std::cout << "Motor 2" << std::endl;
  std::cout << "  Velocity:     " << e.getRightMotor()->getVelocity() << std::endl;
  std::cout << "  Position:     " << e.getRightMotor()->getPosition() << std::endl;
  std::cout << "  Current:      " << e.getRightMotor()->getEffort() << " A" << std::endl;
  if (e.getRightMotor()->get_gains(kp, kd, ki, windup))
  {
    std::cout << "  Gains:        Kp: " << kp << ", Kd: " << kd <<
                 ", Ki: " << ki << ", Windup: " << windup << std::endl;
  }
  std::cout << "Motor Period:   " << e.get_motor_period() << "ms" << std::endl;
  std::cout << "Motor Max Step: " << e.get_motor_max_step() << std::endl;
  std::cout << "IMU" << std::endl;
  std::cout << "  Accel:        " << e.get_imu_acc_x() << ", " <<
                                     e.get_imu_acc_y() << ", " <<
                                     e.get_imu_acc_z() << std::endl;
  std::cout << "  Gyro:         " << e.get_imu_gyro_x() << ", " <<
                                     e.get_imu_gyro_y() << ", " <<
                                     e.get_imu_gyro_z() << std::endl;
  std::cout << "  Magnetometer: " << e.get_imu_mag_x() << ", " <<
                                     e.get_imu_mag_y() << ", " <<
                                     e.get_imu_mag_z() << std::endl;
  std::cout << "Usart3 Baud:    " << e.get_usart3_baud() << std::endl;
  std::cout << "Usart3 Char:    " << e.get_usart3_char() << std::endl;
  std::cout << "Tim12 Mode:     " << e.get_tim12_mode() << std::endl;
  if (e.get_tim12_mode() == 1)
  {
    std::cout << "Tim12 Count:    " << e.get_tim12_count() << std::endl;
  }

  return 0;
}
