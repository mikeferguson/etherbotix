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

#include "rclcpp/rclcpp.hpp"

#include "etherbotix/copy_float.hpp"
#include "etherbotix/dynamixel.hpp"
#include "etherbotix/etherbotix.hpp"
#include "etherbotix/dynamixel_servo.hpp"

namespace etherbotix
{

DynamixelServo::DynamixelServo(const std::string & name, uint8_t id, bool invert)
: name_(name),
  id_(id),
  invert_(invert),
  ticks_(1024),
  range_(dynamixel::AX_SERVO_RANGE),
  rads_per_tick_(range_/ticks_),
  lower_lim_(-dynamixel::AX_SERVO_RANGE / 2.0),
  upper_lim_(dynamixel::AX_SERVO_RANGE / 2.0),
  velocity_lim_(dynamixel::AX_SERVO_MAX_VEL),
  velocity_(0.0),
  position_(0.0),
  effort_(0.0),
  voltage_(-1.0),
  temperature_(-1.0),
  last_update_(0),
  has_desired_position_(false),
  desired_position_(0.0)
{
}

void DynamixelServo::setResolution(int ticks, double range)
{
  ticks_ = ticks;
  center_ = ticks / 2;
  range_ = range;
  rads_per_tick_ = range_ / ticks_;
}

void DynamixelServo::setCenter(int center_ticks)
{
  if (center_ticks < ticks_)
    center_ = center_ticks;
}

void DynamixelServo::setLimits(double lower, double upper, double velocity)
{
  lower_lim_ = lower;
  upper_lim_ = upper;
  velocity_lim_ = velocity;
}

void DynamixelServo::updateFromPacket(int position, uint64_t now)
{
  if (position >= 0 && position < ticks_)
  {
    ++num_reads_;

    // Determine position in radians
    double p = ticksToRads(position);

    // Estimate velocity
    double elapsed = (now - last_update_) * 1e9;
    velocity_ = (p - position_) / elapsed;

    position_ = p;
    last_update_ = now;
  }
  else
  {
    ++num_errors_;
  }
}

void DynamixelServo::updateFromPacket(int voltage, int temperature, uint64_t now)
{
  if (voltage < 255)
  {
    voltage_ = voltage_ / 10.0;
    last_update_ = now;
  }
  if (temperature < 100)
  {
    temperature_ = temperature;
    last_update_ = now;
  }
}

void DynamixelServo::setPosition(double position, double /*velocity*/, double /*effort*/)
{
  desired_position_ = radsToTicks(position);
  has_desired_position_ = true;
}

void DynamixelServo::setVelocity(double /*velocity*/, double /*effort*/)
{
  throw std::runtime_error("velocity control mode is not supported");
}

void DynamixelServo::setEffort(double effort)
{
  (void) effort;
  throw std::runtime_error("effort control mode is not supported");
}

double DynamixelServo::getPositionMin()
{
  return lower_lim_;
}

double DynamixelServo::getPositionMax()
{
  return upper_lim_;
}

double DynamixelServo::getVelocityMax()
{
  return velocity_lim_;
}

double DynamixelServo::ticksToRads(int ticks)
{
  double rads = (ticks - center_) * rads_per_tick_;
  if (invert_)
  {
    return -rads;
  }
  return rads;
}

int DynamixelServo::radsToTicks(double radians)
{
  // Check limits
  if (radians > upper_lim_)
  {
    return radsToTicks(upper_lim_);
  }
  else if (radians < lower_lim_)
  {
    return radsToTicks(lower_lim_);
  }

  // Basic conversion
  int ticks = center_ + (radians / rads_per_tick_);
  if (invert_)
  {
    ticks = center_ - (radians / rads_per_tick_);
  }

  // Check limits
  if (ticks < 0)
  {
    return 0;
  }
  else if (ticks > ticks_)
  {
    return ticks_;
  }

  return ticks;
}

}  // namespace etherbotix
