/*
 * Copyright 2013-2020 Michael E. Ferguson
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

#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "etherbotix/etherbotix_ros.hpp"
#include "etherbotix/copy_float.hpp"
#include "etherbotix/dynamixel.hpp"

namespace etherbotix
{

EtherbotixROS::EtherbotixROS(const rclcpp::NodeOptions & options)
: rclcpp::Node("etherbotix", options),
  Etherbotix(),
  logger_(rclcpp::get_logger("etherbotix")),
  initialized_(false)
{
  // Declare parameters
  ip_ = this->declare_parameter<std::string>("ip_address", ip_);
  port_ = this->declare_parameter<int>("port", port_);
  RCLCPP_INFO(logger_, "Connecting to Etherbotix at %s:%d", ip_.c_str(), port_);

  milliseconds_ = this->declare_parameter<int>("update_interval_ms", 10);

  bool use_imu = this->declare_parameter<bool>("imu", true);
  imu_frame_id_ = this->declare_parameter<std::string>("imu.frame_id", "imu_link");

  // These default to 0.0 - if not set by user they will be set
  // later when we know the version of IMU on the board.
  gyro_scale_ = this->declare_parameter<double>("imu.gyro.scale", 0.0);
  gyro_covariance_ = this->declare_parameter<double>("imu.gyro.covariance", 0.0);
  accel_scale_ = this->declare_parameter<double>("imu.accel.scale", 0.0);
  accel_covariance_ = this->declare_parameter<double>("imu.accel.covariance", 0.0);
  mag_scale_ = this->declare_parameter<double>("imu.mag.scale", 0.0);

  // Setup motors
  std::string l_motor_name = this->declare_parameter<std::string>("l_motor_name",
                                                                  "l_wheel_joint");
  std::string r_motor_name = this->declare_parameter<std::string>("r_motor_name",
                                                                  "r_wheel_joint");
  double ticks_per_radian = this->declare_parameter<double>("ticks_per_radian", 1.0);
  left_motor_ = std::make_shared<EtherbotixMotor>(l_motor_name, ticks_per_radian);
  right_motor_ = std::make_shared<EtherbotixMotor>(r_motor_name, ticks_per_radian);

  double kp = this->declare_parameter<double>("motor_kp", 1.0);
  double kd = this->declare_parameter<double>("motor_kd", 0.0);
  double ki = this->declare_parameter<double>("motor_ki", 0.1);
  double kw = this->declare_parameter<double>("motor_kw", 400.0);
  RCLCPP_INFO(logger_, "Setting gains to %f %f %f %f", kp, kd, ki, kw);
  left_motor_->set_gains(kp, kd, ki, kw);
  right_motor_->set_gains(kp, kd, ki, kw);

  // Controller manager
  controller_manager_ = std::make_shared<robot_controllers_interface::ControllerManager>();
  robot_controllers_interface::JointHandlePtr j;
  j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(left_motor_);
  controller_manager_->addJointHandle(j);
  j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(right_motor_);
  controller_manager_->addJointHandle(j);

  // Dynamixel joints
  std::vector<std::string> joint_names =
    this->declare_parameter<std::vector<std::string>>("servo_joints", std::vector<std::string>());
  servos_.reserve(joint_names.size());
  for (auto joint : joint_names)
  {
    int id = this->declare_parameter<int>(joint + ".id", 1);
    bool invert = this->declare_parameter<bool>(joint + ".invert", false);
    auto servo = std::make_shared<DynamixelServo>(joint, id, invert);

    int ticks = this->declare_parameter<int>(joint + ".ticks", 1024);
    double range = this->declare_parameter<double>(joint + ".range", dynamixel::AX_SERVO_RANGE);
    servo->setResolution(ticks, range);

    int center = this->declare_parameter<int>(joint + ".center", ticks / 2);
    servo->setCenter(center);

    double min_pos = this->declare_parameter<double>(joint + ".min_pos",
                                                     -dynamixel::AX_SERVO_RANGE / 2.0);
    double max_pos = this->declare_parameter<double>(joint + ".max_pos",
                                                     dynamixel::AX_SERVO_RANGE / 2.0);
    double max_vel = this->declare_parameter<double>(joint + ".max_vel",
                                                     dynamixel::AX_SERVO_MAX_VEL);
    servo->setLimits(min_pos, max_pos, max_vel);

    j = std::static_pointer_cast<robot_controllers_interface::JointHandle>(servo);
    controller_manager_->addJointHandle(j);
    servos_.push_back(servo);
    RCLCPP_INFO(logger_, "Adding DynamixelServo: %s with ID %d",
                servo->getName().c_str(), servo->getId());
  }

  // ROS interfaces
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                           std::bind(&EtherbotixROS::publish, this));
  diagnostics_timer_ = this->create_wall_timer(
                         std::chrono::milliseconds(1000),
                         std::bind(&EtherbotixROS::send_diagnostics, this));
  if (use_imu)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/raw", 10);
  }
}

EtherbotixROS::~EtherbotixROS()
{
}

void EtherbotixROS::update(const boost::system::error_code & e)
{
  // Abort if we are shutting down
  if (!rclcpp::ok())
  {
    return;
  }

  if (!initialized_)
  {
    // This has to be done here, shared_from_this doesn't work in constructor
    controller_manager_->init(shared_from_this());
    initialized_ = true;
  }

  // Process messages for servos
  while (rclcpp::ok())
  {
    uint8_t buffer[256];
    uint8_t len = get(buffer);
    if (len == 0)
    {
      break;
    }

    // Parse return packet
    uint8_t id = buffer[dynamixel::PACKET_ID];
    if (id != dynamixel::BROADCAST_ID)
    {
      RCLCPP_ERROR(logger_, "Got unexpected packet for %d", id);
      continue;
    }

    uint64_t now = this->now().nanoseconds();

    uint8_t i = dynamixel::PACKET_PARAM_START;
    for (auto servo : servos_)
    {
      if (i + 1 >= len)
      {
        RCLCPP_ERROR(logger_, "Not enough bytes to update %s", servo->getName().c_str());
        continue;
      }

      uint8_t addr = buffer[dynamixel::PACKET_ASYNC_ADDR];
      if (addr == dynamixel::PRESENT_POSITION_L)
      {
        int position = buffer[i] + (buffer[i+1] << 8);
        servo->updateFromPacket(position, now);
      }
      else if (addr == dynamixel::PRESENT_VOLTAGE)
      {
        servo->updateFromPacket(buffer[i], buffer[i+1], now);
      }
      i += 2;
    }
  }

  // Update controllers
  double seconds = milliseconds_ / 1e3;
  controller_manager_->update(this->now(), rclcpp::Duration::from_seconds(seconds));

  // Generate Commands
  size_t length = 0;
  uint8_t send_buf[4096];

  send_buf[length++] = 0xff;
  send_buf[length++] = 'B';
  send_buf[length++] = 'O';
  send_buf[length++] = 'T';

  // Read 128 bytes from etherbotix
  length += dynamixel::get_read_packet(&send_buf[length], ETHERBOTIX_ID, 0, 128);

  // Get motor update packets
  length += right_motor_->get_packets(&send_buf[length], 2);
  length += left_motor_->get_packets(&send_buf[length], 1);

  // Get servo update packets
  if (!servos_.empty())
  {
    std::vector<uint8_t> servo_ids;

    // Sync write position commands to controlled servos
    std::vector<std::vector<uint8_t>> sync_write;
    for (auto servo : servos_)
    {
      servo_ids.push_back(servo->getId());
      if (servo->hasCommand())
      {
        int pos = servo->getDesiredPosition();
        std::vector<uint8_t> update;
        update.push_back(servo->getId());
        update.push_back(pos % 256);
        update.push_back(pos >> 8);
        sync_write.push_back(update);
      }
    }

    length += dynamixel::get_sync_write_packet(
      &send_buf[length],
      dynamixel::GOAL_POSITION_L,
      sync_write);

    // Sync read position
    length += dynamixel::get_sync_read_packet(
      &send_buf[length],
      servo_ids,
      dynamixel::PRESENT_POSITION_L,
      2);

    // Sync read voltages and temperatures
    length += dynamixel::get_sync_read_packet(
      &send_buf[length],
      servo_ids,
      dynamixel::PRESENT_VOLTAGE,
      2);
  }

  this->send(send_buf, length);

  // Reset timer and async wait
  Etherbotix::update(e);
}

void EtherbotixROS::publish()
{
  if (!rclcpp::ok())
  {
    return;
  }

  auto now = this->now();

  // Publish joint states
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now;
  auto joint_names = controller_manager_->getJointNames();
  for (auto name : joint_names)
  {
    auto j = controller_manager_->getJointHandle(name);
    msg.name.push_back(j->getName());
    msg.position.push_back(j->getPosition());
    msg.velocity.push_back(j->getVelocity());
    msg.effort.push_back(j->getEffort());
  }
  joint_state_pub_->publish(msg);

  // Publish IMU data
  if (imu_pub_ && this->get_imu_version() > 0)
  {
    if (gyro_scale_ == 0.0)
    {
      if (this->get_imu_version() == 2)
      {
        // L3GD20 (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM303DLHC (8g/full scale mode)
        accel_scale_ = 0.0023957;
        accel_covariance_ = 0.34644996;
        // LSM303DLHC (8.1 gauss mode)
        mag_scale_ = 0.0043478;
      }
      else if (this->get_imu_version() == 3)
      {
        // L3GD20H (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM303D (8g/full scale mode)
        accel_scale_ = 0.0023964;
        accel_covariance_ = 0.34644996;
        // LSM303D (12 gauss mode)
        mag_scale_ = 0.000479;
      }
      else  // assuming version == 5
      {
        // LSM6DS33 (2000dps mode)
        gyro_scale_ = 0.001221111;
        gyro_covariance_ = 0.004868938;
        // LSM6DS33 (8g/full scale mode)
        accel_scale_ = 0.0023964;
        accel_covariance_ = 0.34644996;
        // LIS3MDL (12 gauss mode)
        mag_scale_ = 0.000438404;
      }
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.header.stamp = now;

    // Signal no known orientation
    imu_msg.orientation_covariance[0] = -1.0;

    imu_msg.angular_velocity.x = this->get_imu_gyro_x() * gyro_scale_;
    imu_msg.angular_velocity.y = this->get_imu_gyro_y() * gyro_scale_;
    imu_msg.angular_velocity.z = this->get_imu_gyro_z() * gyro_scale_;
    imu_msg.angular_velocity_covariance[0] = gyro_covariance_;
    imu_msg.angular_velocity_covariance[4] = gyro_covariance_;
    imu_msg.angular_velocity_covariance[8] = gyro_covariance_;

    imu_msg.linear_acceleration.x = this->get_imu_acc_x() * accel_scale_;
    imu_msg.linear_acceleration.y = this->get_imu_acc_y() * accel_scale_;
    imu_msg.linear_acceleration.z = this->get_imu_acc_z() * accel_scale_;
    imu_msg.linear_acceleration_covariance[0] = accel_covariance_;
    imu_msg.linear_acceleration_covariance[4] = accel_covariance_;
    imu_msg.linear_acceleration_covariance[8] = accel_covariance_;

    imu_pub_->publish(imu_msg);

    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.frame_id = imu_frame_id_;
    mag_msg.header.stamp = now;

    mag_msg.magnetic_field.x = this->get_imu_mag_x() * mag_scale_;
    mag_msg.magnetic_field.y = this->get_imu_mag_y() * mag_scale_;
    mag_msg.magnetic_field.z = this->get_imu_mag_z() * mag_scale_;

    // TODO(fergs): mag covariance

    mag_pub_->publish(mag_msg);
  }
}

template <typename T>
diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key,
  T value,
  std::string postfix = std::string())
{
  diagnostic_msgs::msg::KeyValue msg;
  msg.key = key;
  std::stringstream ss;
  ss << value;
  ss << postfix;
  msg.value = ss.str();
  return msg;
}

void EtherbotixROS::send_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->now();

  // Add diagnostics for servos
  for (auto servo : servos_)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;

    status.name = servo->getName();
    if (servo->getTemperature() > 60)
    {
      status.message = "OVERHEATED, SHUTDOWN";
      status.level = status.ERROR;
    }
    else if (servo->getTemperature() > 50)
    {
      status.message = "OVERHEATING";
      status.level = status.WARN;
    }
    else
    {
      status.message = "OK";
      status.level = status.OK;
    }

    status.values.push_back(make_key_value("Position", servo->getPosition()));
    status.values.push_back(make_key_value("Temperature", servo->getTemperature()));
    status.values.push_back(make_key_value("Voltage", servo->getVoltage()));
    status.values.push_back(make_key_value("Reads", servo->getNumReads()));
    double error_rate = servo->getNumErrors() / (servo->getNumErrors() + servo->getNumReads());
    status.values.push_back(make_key_value("Error Rate", error_rate));

    msg.status.push_back(status);
  }

  // Add diagnostics for Etherbotix
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "etherbotix";
  status.level = status.OK;
  status.message = "OK";
  status.hardware_id = this->get_unique_id();
  if (this->get_system_voltage() < 10.0)
  {
    status.level = status.ERROR;
    status.message = "Battery depleted!";
  }
  status.values.push_back(make_key_value("Voltage", this->get_system_voltage(), "V"));
  status.values.push_back(make_key_value("Servo Current", this->get_servo_current(), "A"));
  status.values.push_back(make_key_value("Aux. Current", this->get_aux_current(), "A"));
  status.values.push_back(make_key_value("Packets", this->get_packets_recv()));
  status.values.push_back(make_key_value("Packets Bad", this->get_packets_bad()));
  msg.status.push_back(status);

  diagnostics_pub_->publish(msg);
}

}  // namespace etherbotix

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(etherbotix::EtherbotixROS)
