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

#ifndef CMEXA_BASE__SYSTEM_HPP_
#define CMEXA_BASE__SYSTEM_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmexa_base
{
class CmexaBaseBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  CmexaBaseBotSystemHardware();
  RCLCPP_SHARED_PTR_DEFINITIONS(CmexaBaseBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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
  struct WheelEndpoints
  {
    std::string joint_name;
    std::string cmd_topic;
    std::string feedback_topic;
    std::string frame_id;
  };

  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Parameters for the base bot calculation
  double gear_ratio_;
  double wheel_radius_;
  double wheel_separation_x_;
  double wheel_separation_y_;
  double steps_per_revolution_;

  rclcpp::Node::SharedPtr node_;

  // send commands to the hardware
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr command_front_left_pub_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr command_front_right_pub_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr command_rear_left_pub_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr command_rear_right_pub_;

  cmeresearch_msgs::msg::TinkerStepperCommand cmd_message_front_left_;
  cmeresearch_msgs::msg::TinkerStepperCommand cmd_message_rear_left_;
  cmeresearch_msgs::msg::TinkerStepperCommand cmd_message_front_right_;
  cmeresearch_msgs::msg::TinkerStepperCommand cmd_message_rear_right_;

  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_front_left_sub_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_front_right_sub_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_rear_left_sub_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_rear_right_sub_;

  std::mutex feedback_mutex_;
  double front_left_feedback_velocity_steps_s_{0.0};
  double front_right_feedback_velocity_steps_s_{0.0};
  double rear_left_feedback_velocity_steps_s_{0.0};
  double rear_right_feedback_velocity_steps_s_{0.0};

  WheelEndpoints front_left_wheel_;
  WheelEndpoints front_right_wheel_;
  WheelEndpoints rear_left_wheel_;
  WheelEndpoints rear_right_wheel_;

  void feedbackFrontLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg);
  void feedbackFrontRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg);
  void feedbackRearLeftCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg);
  void feedbackRearRightCallback(const cmeresearch_msgs::msg::TinkerStepperFeedback::SharedPtr msg);

};

}  // namespace cmexa_base

#endif // CMEXA_BASE__SYSTEM_HPP_
