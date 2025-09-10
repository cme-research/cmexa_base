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

#include "cmexa_base/mecanumbot_system.hpp"

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

CmexaMecanumBotSystemHardware::CmexaMecanumBotSystemHardware()
{
  node_ = std::make_shared<rclcpp::Node>("cmexa_base");
  command_front_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/front_left/cmd_vel", 10);
  command_front_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/front_right/cmd_vel", 10);
  command_rear_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/rear_left/cmd_vel", 10);
  command_rear_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/rear_right/cmd_vel", 10);

  feedback_front_left_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/front_left/feedback", 10, std::bind(&CmexaMecanumBotSystemHardware::feedbackFrontLeftCallback, this, std::placeholders::_1));
  feedback_front_right_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/front_right/feedback", 10, std::bind(&CmexaMecanumBotSystemHardware::feedbackFrontRightCallback, this, std::placeholders::_1));
  feedback_rear_left_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/rear_left/feedback", 10, std::bind(&CmexaMecanumBotSystemHardware::feedbackRearLeftCallback, this, std::placeholders::_1));
  feedback_rear_right_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/rear_right/feedback", 10, std::bind(&CmexaMecanumBotSystemHardware::feedbackRearRightCallback, this, std::placeholders::_1));
}

void CmexaMecanumBotSystemHardware::feedbackFrontLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
    //RCLCPP_INFO(node_->get_logger(), "FeedbackFrontLeftCallback received: %f", msg->current_velocity);
}

void CmexaMecanumBotSystemHardware::feedbackFrontRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  //RCLCPP_INFO(node_->get_logger(), "FeedbackFrontRightCallback received: %f", msg->current_velocity);
}

void CmexaMecanumBotSystemHardware::feedbackRearLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  //RCLCPP_INFO(node_->get_logger(), "FeedbackRearLeftCallback received: %f", msg->current_velocity);
}

void CmexaMecanumBotSystemHardware::feedbackRearRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
   //RCLCPP_INFO(node_->get_logger(), "FeedbackRearRightCallback received: %f", msg->current_velocity);
}

hardware_interface::CallbackReturn CmexaMecanumBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  wheel_radius_ =
    hardware_interface::stod(info_.hardware_parameters["wheel_radius"]);


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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CmexaMecanumBotSystemHardware::on_configure(
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

hardware_interface::CallbackReturn CmexaMecanumBotSystemHardware::on_activate(
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

hardware_interface::CallbackReturn CmexaMecanumBotSystemHardware::on_deactivate(
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

hardware_interface::return_type CmexaMecanumBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // Read state from hardware
    // TODO: add calculation for odometry??

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Simulate DiffBot wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, get_state(name) + period.seconds() * velo);

      ss << std::endl
         << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
         << "'!";
    }
  }
  //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type cmexa_base ::CmexaMecanumBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  //cmd_message_front_left_.velocity = 123;
  //command_front_left_pub_->publish(cmd_message_front_left_);
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));
    if (name == "front_left_wheel_joint/velocity")
    {
      cmd_message_front_left_.header.stamp = node_->get_clock()->now();
      cmd_message_front_left_.header.frame_id = "front_left_wheel_joint";
      double velocity_in_ms = double(get_command(name));
      // calculate velocity from m/s to rad/s
      double radius = 0.05; //radius of the wheel in meters
      double velocity_in_rads = velocity_in_ms / radius;
      cmd_message_front_left_.velocity = velocity_in_rads;
      command_front_left_pub_->publish(cmd_message_front_left_);
    }
    else if (name == "rear_right_wheel_joint/velocity")
    {
      cmd_message_rear_right_.header.stamp = node_->get_clock()->now();
      cmd_message_rear_right_.header.frame_id = "rear_right_wheel_joint";
      double velocity_in_ms = double(get_command(name));
      // calculate velocity from m/s to rad/s
      double radius = 0.05; //radius of the wheel in meters
      double velocity_in_rads = velocity_in_ms / radius;
      cmd_message_rear_right_.velocity = velocity_in_rads;
      command_rear_right_pub_->publish(cmd_message_rear_right_);
    }
    else if (name == "front_right_wheel_joint/velocity")
    {
      cmd_message_front_right_.header.stamp = node_->get_clock()->now();
      cmd_message_front_right_.header.frame_id = "front_right_wheel_joint";
      double velocity_in_ms = double(get_command(name));
      // calculate velocity from m/s to rad/s
      double radius = 0.05; //radius of the wheel in meters
      double velocity_in_rads = velocity_in_ms / radius;
      cmd_message_front_right_.velocity = velocity_in_rads;
      command_front_right_pub_->publish(cmd_message_front_right_);
    }
    else if (name == "rear_left_wheel_joint/velocity")
    {
      cmd_message_rear_left_.header.stamp = node_->get_clock()->now();
      cmd_message_rear_left_.header.frame_id = "rear_left_wheel_joint";
      double velocity_in_ms = double(get_command(name));
      // calculate velocity from m/s to rad/s
      double radius = 0.05; //radius of the wheel in meters
      double velocity_in_rads = velocity_in_ms / radius;
      cmd_message_rear_left_.velocity = velocity_in_rads;
      command_rear_left_pub_->publish(cmd_message_rear_left_);
    }



    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(name) << " for '" << name << "'!";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace cmexa_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cmexa_base::CmexaMecanumBotSystemHardware, hardware_interface::SystemInterface)
