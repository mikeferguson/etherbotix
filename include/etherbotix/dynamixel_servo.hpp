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

#ifndef ETHERBOTIX__DYNAMIXEL_SERVO_HPP_
#define ETHERBOTIX__DYNAMIXEL_SERVO_HPP_

#include <memory>
#include <string>

#include "robot_controllers_interface/joint_handle.h"

namespace etherbotix
{

class DynamixelServo : public robot_controllers_interface::JointHandle
{
public:
  /**
   * @brief Bridge between etherbotix-connected Dynamixel servos
   *        and robot_controllers package.
   * @param name Name of the servo.
   * @param id The bus ID of the servo.
   * @param invert Whether to invert the position around center.
   */
  DynamixelServo(const std::string & name,
                 uint8_t id,
                 bool invert = false);
  virtual ~DynamixelServo() = default;

  /**
   * @brief Set the resolution parameters of servo.
   *        NOTE: this has to match the hardware, has
   *        no impact on the hardware.
   * @param ticks Number of ticks in position resolution, usually
   *        1024 for AX, or 4096 for MX servos.
   * @param range The range (in degrees) which ticks are spread over.
   */
  void setResolution(int ticks = 1024,
                     double range = 300.0);

  /**
   * @brief Set the number of ticks that corresponds to the center.
   */
  void setCenter(int center_ticks);

  /**
   * @brief Set the limits for commands (in radians)
   */
  void setLimits(double lower, double upper, double velocity);

  // Interface from etherbotix
  uint8_t getId() { return id_; }
  void updateFromPacket(int position, uint64_t now);
  void updateFromPacket(int voltage, int temperature, uint64_t now);
  bool hasCommand() { return has_desired_position_; }
  int getDesiredPosition() { return desired_position_; }

  // This is the JointHandle interface
  virtual std::string getName() { return name_; }
  virtual double getPosition() { return position_; }
  virtual double getVelocity() { return velocity_; }
  virtual double getEffort() { return effort_; }
  virtual void setPosition(double position, double velocity, double effort);
  virtual void setVelocity(double velocity, double effort);
  virtual void setEffort(double effort);
  virtual bool isContinuous() { return false; }
  virtual double getPositionMin();
  virtual double getPositionMax();
  virtual double getVelocityMax();
  virtual double getEffortMax() { return 0.0; }
  virtual void reset() { has_desired_position_ = false; }

  double getVoltage() { return voltage_; }
  double getTemperature() { return temperature_; }
  int getNumReads() { return num_reads_; }
  int getNumErrors() { return num_errors_; }

private:
  double ticksToRads(int ticks);
  int radsToTicks(double radians);

  std::string name_;

  // Servo configuration
  uint8_t id_;
  bool invert_;
  int ticks_;
  int center_;
  double range_;
  double rads_per_tick_;
  double lower_lim_;
  double upper_lim_;
  double velocity_lim_;

  // Statistics
  int num_reads_;
  int num_writes_;
  int num_errors_;

  // Processed feedback
  double velocity_;  // rad/s
  double position_;  // radians
  double effort_;    // ?
  double voltage_;
  double temperature_;
  uint64_t last_update_;

  bool has_desired_position_;
  int desired_position_;
};

using DynamixelServoPtr = std::shared_ptr<DynamixelServo>;

}  // namespace etherbotix

#endif  // ETHERBOTIX__DYNAMIXEL_SERVO_HPP_
