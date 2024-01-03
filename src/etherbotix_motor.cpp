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

#include <string>
#include <stdexcept>
#include <vector>

#include "etherbotix/copy_float.hpp"
#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"
#include "etherbotix/etherbotix_motor.hpp"

namespace etherbotix
{

EtherbotixMotor::EtherbotixMotor(const std::string& name, double ticks_per_radian)
: name_(name),
  ticks_per_radian_(ticks_per_radian),
  ticks_offset_(0),
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

void EtherbotixMotor::set_ticks_per_radian(double ticks_per_radian)
{
  ticks_per_radian_ = ticks_per_radian;
}

void EtherbotixMotor::set_ticks_offset(int ticks_offset)
{
  ticks_offset_ = ticks_offset;
}

bool EtherbotixMotor::set_gains(float kp, float kd, float ki, float windup)
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

void EtherbotixMotor::update_from_packet(int16_t velocity, int32_t position, int16_t current)
{
  if (motor_period_ < 0)
  {
    throw std::runtime_error("motor_period has not yet been set");
  }

  // Convert to radians/second
  velocity_ = (velocity / ticks_per_radian_) * (1000.0 / motor_period_);

  // Convert to radians
  position_ = (position - ticks_offset_) / ticks_per_radian_;

  // TODO(fergs): Convert to amperes
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

  uint8_t len = dynamixel::get_write_packet(
    buffer,
    Etherbotix::ETHERBOTIX_ID,
    (motor_idx == 1) ? Etherbotix::REG_MOTOR1_VEL : Etherbotix::REG_MOTOR2_VEL,
    {static_cast<uint8_t>(desired_velocity_ & 0xff),
     static_cast<uint8_t>(desired_velocity_ >> 8)});

  // Update gains if needed
  if (kp_ != desired_kp_ ||
      kd_ != desired_kd_ ||
      ki_ != desired_ki_ ||
      windup_ != desired_windup_)
  {
    if (kp_ >= 0.0 && kd_ >= 0.0 && ki_ >= 0.0 &&
        desired_kp_ >= 0.0 && desired_kd_ >= 0.0 && desired_ki_ >= 0.0)
    {
      std::vector<uint8_t> params;
      copy_float(desired_kp_, params);
      copy_float(desired_kd_, params);
      copy_float(desired_ki_, params);
      copy_float(desired_windup_, params);
      len += dynamixel::get_write_packet(
        &buffer[len],
        Etherbotix::ETHERBOTIX_ID,
        (motor_idx == 1) ? Etherbotix::REG_MOTOR1_KP : Etherbotix::REG_MOTOR2_KP,
        params);
    }
  }

  return len;
}

void EtherbotixMotor::setPosition(double /*position*/, double /*velocity*/, double /*effort*/)
{
  throw std::runtime_error("position control mode is not supported");
}

void EtherbotixMotor::setVelocity(double velocity, double /*effort*/)
{
  if (motor_period_ < 0)
  {
    return;
  }

  desired_velocity_ = (velocity * ticks_per_radian_) * (motor_period_ / 1000.0);
}

void EtherbotixMotor::setEffort(double effort)
{
  (void) effort;
  throw std::runtime_error("effort control mode is not supported");
}

double EtherbotixMotor::getVelocityMax()
{
  return 10000;  // TODO(fergs): load this value
}

}  // namespace etherbotix
