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

#include <string>

namespace etherbotix
{

class EtherbotixMotor
{
public:
  /**
   * @brief Manager for one of the base motors on an Etherbotix.
   * @param name Name of the motor (will also be name of joint).
   */
  EtherbotixMotor(const std::string& name, double ticks_per_radian);
  virtual ~EtherbotixMotor();

  std::string get_name() { return name_; }

  /** @brief Get the position in radians. */
  double get_position() { return position_; }

  /** @brief Get the velocity in rad/sec. */
  double get_velocity() { return velocity_; }

  /** @brief Get the motor current in amperes. */
  float get_current() { return current_; }

  /** @brief Send gains to the board. */
  bool set_gain(float kp, float kd, float ki, float windup);

  /** @brief Get the gains. Returns false if no comms. */
  bool get_gains(float & kp, float & kd, float & ki, float & windup);

  /** @brief Set a velocity command. */
  bool set_command(double velocity);

  /** @brief Set status read from board. */
  void update_from_packet(int16_t velocity, int32_t position, int16_t current);

  /** @brief Set the motor period read from board. */
  void update_motor_period_from_packet(uint8_t period);

  /** @brief Set the gains read from board. */
  void update_gains_from_packet(float kp, float kd, float ki, float windup);

  /** @brief Get command packet(s). */
  uint8_t get_packets(uint8_t * buffer, int motor_idx);

private:
  std::string name_;
  double ticks_per_radian_;
  int motor_period_;

  double velocity_;  // rad/s
  double position_;  // radians
  float current_;    // amperes

  float kp_;
  float kd_;
  float ki_;
  float windup_;

  int16_t desired_velocity_;
  float desired_kp_;
  float desired_kd_;
  float desired_ki_;
  float desired_windup_;
};

}  // namespace etherbotix

#endif  // ETHERBOTIX__ETHERBOTIX_MOTOR_HPP_
