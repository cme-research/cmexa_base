// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cmexa_base/cmexa_base_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmexa_base
{
namespace
{
std::string get_hardware_param_or_default(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : default_value;
}
}  // namespace

CmexaBaseBotSystemHardware::CmexaBaseBotSystemHardware()
{
}

void CmexaBaseBotSystemHardware::feedbackFrontLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  front_left_feedback_velocity_steps_s_ = msg->current_velocity;
}

void CmexaBaseBotSystemHardware::feedbackFrontRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  front_right_feedback_velocity_steps_s_ = msg->current_velocity;
}

void CmexaBaseBotSystemHardware::feedbackRearLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  rear_left_feedback_velocity_steps_s_ = msg->current_velocity;
}

void CmexaBaseBotSystemHardware::feedbackRearRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  rear_right_feedback_velocity_steps_s_ = msg->current_velocity;
}

hardware_interface::CallbackReturn CmexaBaseBotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("example_param_hw_start_duration_sec") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("example_param_hw_stop_duration_sec") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("gear_ratio") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("wheel_radius") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("wheel_separation_x") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("wheel_separation_y") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("steps_per_revolution") == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(get_logger(), "Missing hardware parameter(s)!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  gear_ratio_ = hardware_interface::stod(info_.hardware_parameters["gear_ratio"]);
  wheel_radius_ = hardware_interface::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_x_ = hardware_interface::stod(info_.hardware_parameters["wheel_separation_x"]);
  wheel_separation_y_ = hardware_interface::stod(info_.hardware_parameters["wheel_separation_y"]);
  steps_per_revolution_ = hardware_interface::stod(info_.hardware_parameters["steps_per_revolution"]);
  // Bricklet microstep mode (1, 2, 4, 8, 16, 32, 64, 128 or 256). Falls back to 8
  // when the URDF predates the parameter — matches the SILENT_STEPPER_V2_STEP_RESOLUTION_8
  // hardcoded in cmeresearch_stepper_driver.
  step_resolution_ = hardware_interface::stod(
    get_hardware_param_or_default(info_, "step_resolution", "8"));

  // Optional per-wheel velocity ceiling. When the URDF passes max_step_vel
  // (microsteps/s on the motor shaft — the same value handed to
  // cmeresearch_stepper_driver), derive the equivalent wheel angular velocity
  // and enable proportional normalization in write(). Mirrors the driver-side
  // clamp: wheel_rad/s = microsteps/s * 2*pi / (steps_per_rev * step_res * gear).
  // Omitting the parameter leaves normalization disabled (0.0) and preserves
  // the previous behaviour of relying solely on the bricklet clamp.
  const std::string max_step_vel_str =
    get_hardware_param_or_default(info_, "max_step_vel", "");
  if (!max_step_vel_str.empty()) {
    const double max_step_vel = hardware_interface::stod(max_step_vel_str);
    const double denom = steps_per_revolution_ * step_resolution_ * gear_ratio_;
    if (max_step_vel > 0.0 && denom > 0.0) {
      max_wheel_velocity_rad_s_ = max_step_vel * (2.0 * M_PI) / denom;
      RCLCPP_INFO(
        get_logger(),
        "Wheel-velocity normalization enabled: max_step_vel=%.0f -> %.3f wheel rad/s",
        max_step_vel, max_wheel_velocity_rad_s_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "max_step_vel='%s' with denom=%.3f is invalid; normalization disabled.",
        max_step_vel_str.c_str(), denom);
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "max_step_vel not set; per-wheel velocity normalization disabled "
      "(relying on the stepper driver clamp only).");
  }

  node_ = get_node();
  if (!node_) {
    RCLCPP_FATAL(get_logger(), "Hardware interface node not available.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  front_left_wheel_.joint_name =
    get_hardware_param_or_default(info_, "front_left_joint", "front_left_wheel_joint");
  front_right_wheel_.joint_name =
    get_hardware_param_or_default(info_, "front_right_joint", "front_right_wheel_joint");
  rear_left_wheel_.joint_name =
    get_hardware_param_or_default(info_, "rear_left_joint", "rear_left_wheel_joint");
  rear_right_wheel_.joint_name =
    get_hardware_param_or_default(info_, "rear_right_joint", "rear_right_wheel_joint");

  front_left_wheel_.cmd_topic =
    get_hardware_param_or_default(info_, "front_left_cmd_topic", "~/front_left/cmd_vel");
  front_right_wheel_.cmd_topic =
    get_hardware_param_or_default(info_, "front_right_cmd_topic", "~/front_right/cmd_vel");
  rear_left_wheel_.cmd_topic =
    get_hardware_param_or_default(info_, "rear_left_cmd_topic", "~/rear_left/cmd_vel");
  rear_right_wheel_.cmd_topic =
    get_hardware_param_or_default(info_, "rear_right_cmd_topic", "~/rear_right/cmd_vel");

  front_left_wheel_.feedback_topic =
    get_hardware_param_or_default(info_, "front_left_feedback_topic", "~/front_left/feedback");
  front_right_wheel_.feedback_topic =
    get_hardware_param_or_default(info_, "front_right_feedback_topic", "~/front_right/feedback");
  rear_left_wheel_.feedback_topic =
    get_hardware_param_or_default(info_, "rear_left_feedback_topic", "~/rear_left/feedback");
  rear_right_wheel_.feedback_topic =
    get_hardware_param_or_default(info_, "rear_right_feedback_topic", "~/rear_right/feedback");

  front_left_wheel_.frame_id =
    get_hardware_param_or_default(info_, "front_left_frame_id", front_left_wheel_.joint_name);
  front_right_wheel_.frame_id =
    get_hardware_param_or_default(info_, "front_right_frame_id", front_right_wheel_.joint_name);
  rear_left_wheel_.frame_id =
    get_hardware_param_or_default(info_, "rear_left_frame_id", rear_left_wheel_.joint_name);
  rear_right_wheel_.frame_id =
    get_hardware_param_or_default(info_, "rear_right_frame_id", rear_right_wheel_.joint_name);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto has_velocity_and_position = [this](const std::string & joint_name) {
      const std::string velocity = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      const std::string position = joint_name + "/" + hardware_interface::HW_IF_POSITION;
      return joint_state_interfaces_.find(velocity) != joint_state_interfaces_.end() &&
             joint_state_interfaces_.find(position) != joint_state_interfaces_.end() &&
             joint_command_interfaces_.find(velocity) != joint_command_interfaces_.end();
    };

  if (!has_velocity_and_position(front_left_wheel_.joint_name) ||
      !has_velocity_and_position(front_right_wheel_.joint_name) ||
      !has_velocity_and_position(rear_left_wheel_.joint_name) ||
      !has_velocity_and_position(rear_right_wheel_.joint_name))
  {
    RCLCPP_FATAL(
      get_logger(),
      "Wheel joint mapping is invalid. Verify *_joint hardware parameters and joint interfaces.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_front_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>(
    front_left_wheel_.cmd_topic, rclcpp::QoS(10));
  command_front_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>(
    front_right_wheel_.cmd_topic, rclcpp::QoS(10));
  command_rear_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>(
    rear_left_wheel_.cmd_topic, rclcpp::QoS(10));
  command_rear_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>(
    rear_right_wheel_.cmd_topic, rclcpp::QoS(10));

  feedback_front_left_sub_ =
    node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>(
    front_left_wheel_.feedback_topic, rclcpp::QoS(10),
    std::bind(&CmexaBaseBotSystemHardware::feedbackFrontLeftCallback, this, std::placeholders::_1));
  feedback_front_right_sub_ =
    node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>(
    front_right_wheel_.feedback_topic, rclcpp::QoS(10),
    std::bind(&CmexaBaseBotSystemHardware::feedbackFrontRightCallback, this, std::placeholders::_1));
  feedback_rear_left_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>(
    rear_left_wheel_.feedback_topic, rclcpp::QoS(10),
    std::bind(&CmexaBaseBotSystemHardware::feedbackRearLeftCallback, this, std::placeholders::_1));
  feedback_rear_right_sub_ =
    node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>(
    rear_right_wheel_.feedback_topic, rclcpp::QoS(10),
    std::bind(&CmexaBaseBotSystemHardware::feedbackRearRightCallback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CmexaBaseBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CmexaBaseBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CmexaBaseBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CmexaBaseBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TinkerStepperFeedback.current_velocity is microsteps/s on the motor shaft.
  // wheel_rad/s = microsteps/s × 2π / (steps_per_rev × step_resolution × gear_ratio)
  const double rad_per_microstep =
    (2.0 * M_PI) / (steps_per_revolution_ * step_resolution_ * gear_ratio_);

  double fl_steps = 0.0;
  double fr_steps = 0.0;
  double rl_steps = 0.0;
  double rr_steps = 0.0;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    fl_steps = front_left_feedback_velocity_steps_s_;
    fr_steps = front_right_feedback_velocity_steps_s_;
    rl_steps = rear_left_feedback_velocity_steps_s_;
    rr_steps = rear_right_feedback_velocity_steps_s_;
  }

  double fl_vel = fl_steps * rad_per_microstep;
  double fr_vel = fr_steps * rad_per_microstep;
  double rl_vel = rl_steps * rad_per_microstep;
  double rr_vel = rr_steps * rad_per_microstep;

  // Update hardware interface states
  set_state(front_left_wheel_.joint_name + "/velocity", fl_vel);
  set_state(front_right_wheel_.joint_name + "/velocity", fr_vel);
  set_state(rear_left_wheel_.joint_name + "/velocity", rl_vel);
  set_state(rear_right_wheel_.joint_name + "/velocity", rr_vel);

  // Integrate position
  set_state(
    front_left_wheel_.joint_name + "/position",
    get_state(front_left_wheel_.joint_name + "/position") + fl_vel * period.seconds());
  set_state(
    front_right_wheel_.joint_name + "/position",
    get_state(front_right_wheel_.joint_name + "/position") + fr_vel * period.seconds());
  set_state(
    rear_left_wheel_.joint_name + "/position",
    get_state(rear_left_wheel_.joint_name + "/position") + rl_vel * period.seconds());
  set_state(
    rear_right_wheel_.joint_name + "/position",
    get_state(rear_right_wheel_.joint_name + "/position") + rr_vel * period.seconds());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CmexaBaseBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read all four wheel velocity commands (wheel rad/s) up front so they can be
  // normalized as a set. On a mecanum base each wheel demand is a signed sum of
  // vx, vy and wz*L, so a diagonal or translation+rotation command drives two
  // wheels far above the per-axis twist limit. Clamping wheels independently
  // (as the downstream bricklet does) would saturate those two while leaving
  // the others untouched, distorting the commanded motion direction. Instead we
  // scale all four by a single factor so the fastest wheel just meets the
  // ceiling and the motion vector is preserved.
  double fl = get_command(front_left_wheel_.joint_name + "/velocity");
  double fr = get_command(front_right_wheel_.joint_name + "/velocity");
  double rl = get_command(rear_left_wheel_.joint_name + "/velocity");
  double rr = get_command(rear_right_wheel_.joint_name + "/velocity");

  if (max_wheel_velocity_rad_s_ > 0.0)
  {
    const double max_mag = std::max(
      {std::abs(fl), std::abs(fr), std::abs(rl), std::abs(rr)});
    if (max_mag > max_wheel_velocity_rad_s_)
    {
      const double scale = max_wheel_velocity_rad_s_ / max_mag;
      fl *= scale;
      fr *= scale;
      rl *= scale;
      rr *= scale;
    }
  }

  const auto now = node_->get_clock()->now();

  cmd_message_front_left_.header.stamp = now;
  cmd_message_front_left_.header.frame_id = front_left_wheel_.frame_id;
  cmd_message_front_left_.velocity = fl;
  command_front_left_pub_->publish(cmd_message_front_left_);

  cmd_message_front_right_.header.stamp = now;
  cmd_message_front_right_.header.frame_id = front_right_wheel_.frame_id;
  cmd_message_front_right_.velocity = fr;
  command_front_right_pub_->publish(cmd_message_front_right_);

  cmd_message_rear_left_.header.stamp = now;
  cmd_message_rear_left_.header.frame_id = rear_left_wheel_.frame_id;
  cmd_message_rear_left_.velocity = rl;
  command_rear_left_pub_->publish(cmd_message_rear_left_);

  cmd_message_rear_right_.header.stamp = now;
  cmd_message_rear_right_.header.frame_id = rear_right_wheel_.frame_id;
  cmd_message_rear_right_.velocity = rr;
  command_rear_right_pub_->publish(cmd_message_rear_right_);

  return hardware_interface::return_type::OK;
}

}  // namespace cmexa_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cmexa_base::CmexaBaseBotSystemHardware, hardware_interface::SystemInterface)
