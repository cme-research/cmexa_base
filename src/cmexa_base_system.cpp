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

CmexaBaseBotSystemHardware::CmexaBaseBotSystemHardware()
{
  node_ = std::make_shared<rclcpp::Node>("cmexa_base");
  command_front_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/front_left/cmd_vel", 10);
  command_front_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/front_right/cmd_vel", 10);
  command_rear_left_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/rear_left/cmd_vel", 10);
  command_rear_right_pub_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperCommand>("~/rear_right/cmd_vel", 10);

  feedback_front_left_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/front_left/feedback", 10, std::bind(&CmexaBaseBotSystemHardware::feedbackFrontLeftCallback, this, std::placeholders::_1));
  feedback_front_right_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/front_right/feedback", 10, std::bind(&CmexaBaseBotSystemHardware::feedbackFrontRightCallback, this, std::placeholders::_1));
  feedback_rear_left_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/rear_left/feedback", 10, std::bind(&CmexaBaseBotSystemHardware::feedbackRearLeftCallback, this, std::placeholders::_1));
  feedback_rear_right_sub_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>("~/rear_right/feedback", 10, std::bind(&CmexaBaseBotSystemHardware::feedbackRearRightCallback, this, std::placeholders::_1));
}

void CmexaBaseBotSystemHardware::feedbackFrontLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  feedback_front_left_msg_ = *msg;
}

void CmexaBaseBotSystemHardware::feedbackFrontRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  feedback_front_right_msg_ = *msg;
}

void CmexaBaseBotSystemHardware::feedbackRearLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  feedback_rear_left_msg_ = *msg;
}

void CmexaBaseBotSystemHardware::feedbackRearRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg)
{
  feedback_rear_right_msg_ = *msg;
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

  if (!node_) {
    RCLCPP_FATAL(get_logger(), "Node not initialized!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  odometry_x_ = 0.0;
  odometry_y_ = 0.0;
  odometry_theta_ = 0.0;


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
  // Update joint states from feedback
  // Note: TinkerStepperFeedback.current_velocity is in steps/s
  // We convert it to rad/s for joint states
  double rad_per_step = (2.0 * M_PI) / (steps_per_revolution_ * gear_ratio_);

  double fl_vel = feedback_front_left_msg_.current_velocity * rad_per_step;
  double fr_vel = feedback_front_right_msg_.current_velocity * rad_per_step;
  double rl_vel = feedback_rear_left_msg_.current_velocity * rad_per_step;
  double rr_vel = feedback_rear_right_msg_.current_velocity * rad_per_step;

  // Update hardware interface states
  set_state("front_left_wheel_joint/velocity", fl_vel);
  set_state("front_right_wheel_joint/velocity", fr_vel);
  set_state("rear_left_wheel_joint/velocity", rl_vel);
  set_state("rear_right_wheel_joint/velocity", rr_vel);

  // Integrate position
  set_state("front_left_wheel_joint/position", get_state("front_left_wheel_joint/position") + fl_vel * period.seconds());
  set_state("front_right_wheel_joint/position", get_state("front_right_wheel_joint/position") + fr_vel * period.seconds());
  set_state("rear_left_wheel_joint/position", get_state("rear_left_wheel_joint/position") + rl_vel * period.seconds());
  set_state("rear_right_wheel_joint/position", get_state("rear_right_wheel_joint/position") + rr_vel * period.seconds());

  // Calculate and publish odometry
  updateOdometry(period);

  return hardware_interface::return_type::OK;
}

void CmexaBaseBotSystemHardware::updateOdometry(const rclcpp::Duration & period)
{
  double fl_vel = get_state("front_left_wheel_joint/velocity");
  double fr_vel = get_state("front_right_wheel_joint/velocity");
  double rl_vel = get_state("rear_left_wheel_joint/velocity");
  double rr_vel = get_state("rear_right_wheel_joint/velocity");

  // Mecanum kinematics (Assuming standard layout)
  // vx = (fl + fr + rl + rr) * r / 4
  // vy = (-fl + fr + rl - rr) * r / 4
  // wz = (-fl + fr - rl + rr) * r / (4 * (lx + ly))
  double r = wheel_radius_;
  double lx = wheel_separation_x_ / 2.0;
  double ly = wheel_separation_y_ / 2.0;

  double vx = (fl_vel + fr_vel + rl_vel + rr_vel) * r / 4.0;
  double vy = (-fl_vel + fr_vel + rl_vel - rr_vel) * r / 4.0;
  double wz = (-fl_vel + fr_vel - rl_vel + rr_vel) * r / (4.0 * (lx + ly));

  // Integrate pose
  double dt = period.seconds();
  odometry_x_ += (vx * cos(odometry_theta_) - vy * sin(odometry_theta_)) * dt;
  odometry_y_ += (vx * sin(odometry_theta_) + vy * cos(odometry_theta_)) * dt;
  odometry_theta_ += wz * dt;

  // Publish Odometry message
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp = node_->get_clock()->now();
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";

  odom_msg->pose.pose.position.x = odometry_x_;
  odom_msg->pose.pose.position.y = odometry_y_;
  odom_msg->pose.pose.orientation.z = sin(odometry_theta_ / 2.0);
  odom_msg->pose.pose.orientation.w = cos(odometry_theta_ / 2.0);

  odom_msg->twist.twist.linear.x = vx;
  odom_msg->twist.twist.linear.y = vy;
  odom_msg->twist.twist.angular.z = wz;

  odom_pub_->publish(std::move(odom_msg));

  // Publish TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = node_->get_clock()->now();
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = odometry_x_;
  tf_msg.transform.translation.y = odometry_y_;
  tf_msg.transform.rotation.z = sin(odometry_theta_ / 2.0);
  tf_msg.transform.rotation.w = cos(odometry_theta_ / 2.0);

  tf_broadcaster_->sendTransform(tf_msg);
}

hardware_interface::return_type cmexa_base ::CmexaBaseBotSystemHardware::write(
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

      cmd_message_front_left_.velocity = double(get_command(name)) * gear_ratio_;
      command_front_left_pub_->publish(cmd_message_front_left_);
    }
    else if (name == "rear_right_wheel_joint/velocity")
    {
      cmd_message_rear_right_.header.stamp = node_->get_clock()->now();
      cmd_message_rear_right_.header.frame_id = "rear_right_wheel_joint";

      cmd_message_rear_right_.velocity = double(get_command(name)) * gear_ratio_;
      command_rear_right_pub_->publish(cmd_message_rear_right_);
    }
    else if (name == "front_right_wheel_joint/velocity")
    {
      cmd_message_front_right_.header.stamp = node_->get_clock()->now();
      cmd_message_front_right_.header.frame_id = "front_right_wheel_joint";

      cmd_message_front_right_.velocity = double(get_command(name)) * gear_ratio_;
      command_front_right_pub_->publish(cmd_message_front_right_);
    }
    else if (name == "rear_left_wheel_joint/velocity")
    {
      cmd_message_rear_left_.header.stamp = node_->get_clock()->now();
      cmd_message_rear_left_.header.frame_id = "rear_left_wheel_joint";

      cmd_message_rear_left_.velocity = double(get_command(name)) * gear_ratio_;
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
  cmexa_base::CmexaBaseBotSystemHardware, hardware_interface::SystemInterface)
