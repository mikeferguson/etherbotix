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

#ifndef ETHERBOTIX__ETHERBOTIX_MOTOR_HPP_
#define ETHERBOTIX__ETHERBOTIX_MOTOR_HPP_

#include <memory>
#include <string>

#include "robot_controllers_interface/joint_handle.h"

namespace etherbotix
{

class EtherbotixMotor : public robot_controllers_interface::JointHandle
{
  static constexpr uint8_t MODE_VELOCITY = 1;
  static constexpr uint8_t MODE_POSITION = 2;

public:
  /**
   * @brief Manager for one of the base motors on an Etherbotix.
   * @param name Name of the motor (will also be name of joint).
   */
  EtherbotixMotor(const std::string& name, double ticks_per_radian);
  virtual ~EtherbotixMotor();

  virtual std::string getName() { return name_; }

  /** @brief Get the position in radians. */
  virtual double getPosition() { return position_; }

  /** @brief Get the velocity in rad/sec. */
  virtual double getVelocity() { return velocity_; }

  /** @brief Get the motor current in amperes. */
  virtual double getEffort() { return current_; }

  /** @brief Set the ticks per radians for conversion. */
  void set_ticks_per_radian(double ticks_per_radian);

  /** @brief Set the ticks offset for zero radians. */
  void set_ticks_offset(int ticks_offset);

  /** @brief Set the limits of the joint. */
  void set_limits(double position_min, double position_max, double velocity_max,
                  double effort_max_, bool continuous);

  /** @brief Send gains to the board. */
  bool set_gains(float kp, float kd, float ki, float windup);

  /** @brief Get the gains. Returns false if no comms. */
  bool get_gains(float & kp, float & kd, float & ki, float & windup);

  /** @brief Set status read from board. */
  void update_from_packet(int16_t velocity, int32_t position, int16_t current);

  /** @brief Set the motor period read from board. */
  void update_motor_period_from_packet(uint8_t period);

  /** @brief Set the gains read from board. */
  void update_gains_from_packet(float kp, float kd, float ki, float windup);

  /** @brief Get command packet(s). */
  uint8_t get_packets(uint8_t * buffer, int motor_idx);

  // This is the JointHandle interface
  virtual void setPosition(double position, double velocity, double effort);
  virtual void setVelocity(double velocity, double effort);
  virtual void setEffort(double effort);
  virtual bool isContinuous() { return is_continuous_; }
  virtual double getPositionMin() { return position_min_; }
  virtual double getPositionMax() { return position_max_; }
  virtual double getVelocityMax() { return velocity_max_; }
  virtual double getEffortMax() { return effort_max_; }
  virtual void reset();

private:
  std::string name_;
  double ticks_per_radian_;
  int ticks_offset_;
  int motor_period_;

  double velocity_;  // rad/s
  double position_;  // radians
  double current_;   // amperes

  float kp_;
  float kd_;
  float ki_;
  float windup_;

  int16_t desired_velocity_;
  int32_t desired_position_;
  uint8_t desired_mode_;
  float desired_kp_;
  float desired_kd_;
  float desired_ki_;
  float desired_windup_;

  double position_min_, position_max_;
  double velocity_max_;
  double effort_max_;
  bool is_continuous_;
};

using EtherbotixMotorPtr = std::shared_ptr<EtherbotixMotor>;

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_MOTOR_HPP_
