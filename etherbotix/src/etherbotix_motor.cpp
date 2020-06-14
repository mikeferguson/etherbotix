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

#include <stdexcept>

#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"
#include "etherbotix/etherbotix_motor.hpp"

#include <iostream>

namespace etherbotix
{

EtherbotixMotor::EtherbotixMotor(const std::string& name, double ticks_per_radian)
: name_(name),
  ticks_per_radian_(ticks_per_radian),
  motor_period_(-1),
  velocity_(0.0),
  position_(0.0),
  current_(0.0),
  kp_(-1.0),
  kd_(-1.0),
  ki_(-1.0),
  windup_(-1.0),
  desired_velocity_(0.0),
  desired_kp_(-1.0),
  desired_kd_(-1.0),
  desired_ki_(-1.0),
  desired_windup_(-1.0)
{

}

EtherbotixMotor::~EtherbotixMotor(){}

bool EtherbotixMotor::set_gain(float kp, float kd, float ki, float windup)
{
  desired_kp_ = kp;
  desired_kd_ = kd;
  desired_ki_ = ki;
  desired_windup_ = windup;

  // Always accept updates
  return true;
}

bool EtherbotixMotor::get_gains(float & kp, float & kd, float & ki, float & windup)
{
  if (kp_ < 0 || kd_ < 0 || ki_ < 0)
  {
    // Not yet read from board
    return false;
  }

  kp = kp_;
  kd = kd_;
  ki = ki_;
  windup = windup_;
  return true;
}

bool EtherbotixMotor::set_command(double velocity)
{
  if (motor_period_ < 0)
  {
    return false;
  }

  desired_velocity_ = (velocity / ticks_per_radian_) * (motor_period_ / 1000.0);
  return true;
}

void EtherbotixMotor::update_from_packet(int16_t velocity, int32_t position, int16_t current)
{
  if (motor_period_ < 0)
  {
    throw std::runtime_error("motor_period has not yet been set");
  }

  // Convert to radians/second
  velocity_ = (velocity / ticks_per_radian_) * (1000.0 / motor_period_);

  // Convert to radians
  position_ = position / ticks_per_radian_;

  // TODO: Convert to amperes
  current_ = current;
}

void EtherbotixMotor::update_motor_period_from_packet(uint8_t period)
{
  motor_period_ = period;
}

void EtherbotixMotor::update_gains_from_packet(float kp, float kd, float ki, float windup)
{
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
  windup_ = windup;
}

uint8_t EtherbotixMotor::get_packets(uint8_t * buffer, int motor_idx)
{
  if (motor_period_ < 0)
  {
    // Not initialized
    return 0;
  }

  uint8_t len = 0;
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  buffer[len++] = Etherbotix::ETHERBOTIX_ID;
  buffer[len++] = 5;  // Length of remaining packet
  buffer[len++] = dynamixel::AX_WRITE_DATA;
  if (motor_idx == 1)
  {
    buffer[len++] = Etherbotix::REG_MOTOR1_VEL;
  }
  else
  {
    buffer[len++] = Etherbotix::REG_MOTOR2_VEL;
  }
  buffer[len++] = (desired_velocity_ & 0xff);
  buffer[len++] = (desired_velocity_ >> 8);
  buffer[len++] = dynamixel::compute_checksum(buffer, 9);

  // TODO: update gains if needed

  return len;
}

}  // namespace etherbotix
