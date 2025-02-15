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

#ifndef UNICO_BASE__DIFFBOT_SYSTEM_HPP_
#define UNICO_BASE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unico_msgs/msg/unico_stepper_command.hpp"
#include "unico_msgs/msg/unico_stepper_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unico_base
{
class UnicoDiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  UnicoDiffBotSystemHardware();
  RCLCPP_SHARED_PTR_DEFINITIONS(UnicoDiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // change to get node_
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  rclcpp::Node::SharedPtr node_;

  // send commands to the hardware
  rclcpp::Publisher<unico_msgs::msg::UnicoStepperCommand>::SharedPtr command_front_left_pub_;
  rclcpp::Publisher<unico_msgs::msg::UnicoStepperCommand>::SharedPtr command_front_right_pub_;
  rclcpp::Publisher<unico_msgs::msg::UnicoStepperCommand>::SharedPtr command_rear_left_pub_;
  rclcpp::Publisher<unico_msgs::msg::UnicoStepperCommand>::SharedPtr command_rear_right_pub_;

  unico_msgs::msg::UnicoStepperCommand cmd_message_front_left_;
  unico_msgs::msg::UnicoStepperCommand cmd_message_rear_left_;
  unico_msgs::msg::UnicoStepperCommand cmd_message_front_right_;
  unico_msgs::msg::UnicoStepperCommand cmd_message_rear_right_;

  rclcpp::Subscription<unico_msgs::msg::UnicoStepperFeedback>::SharedPtr feedback_front_left_sub_;
  rclcpp::Subscription<unico_msgs::msg::UnicoStepperFeedback>::SharedPtr feedback_front_right_sub_;
  rclcpp::Subscription<unico_msgs::msg::UnicoStepperFeedback>::SharedPtr feedback_rear_left_sub_;
  rclcpp::Subscription<unico_msgs::msg::UnicoStepperFeedback>::SharedPtr feedback_rear_right_sub_;

	//TODO: std::shared_ptr<unico_msgs::msg::UnicoStepperFeedback> feedback_front_left_msg_;
  unico_msgs::msg::UnicoStepperFeedback feedback_front_left_msg_;
  unico_msgs::msg::UnicoStepperFeedback feedback_front_right_msg_;
  unico_msgs::msg::UnicoStepperFeedback feedback_rear_left_msg_;
  unico_msgs::msg::UnicoStepperFeedback feedback_rear_right_msg_;

  void feedbackFrontLeftCallback(const unico_msgs::msg::UnicoStepperFeedback::SharedPtr msg);
  void feedbackFrontRightCallback(const unico_msgs::msg::UnicoStepperFeedback::SharedPtr msg);
  void feedbackRearLeftCallback(const unico_msgs::msg::UnicoStepperFeedback::SharedPtr msg);
  void feedbackRearRightCallback(const unico_msgs::msg::UnicoStepperFeedback::SharedPtr msg);

};

}  // namespace unico_base

#endif  // UNICO_BASE__DIFFBOT_SYSTEM_HPP_
