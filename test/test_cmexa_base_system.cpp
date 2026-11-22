// Copyright 2026 CME Research
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

// Loads the real cmexa_base plugin through ros2_control's ResourceManager and
// checks the conversion between joint interfaces and the stepper driver topics:
//   read():  TinkerStepperFeedback (signed microsteps/s) -> joint velocity/position
//   write(): joint velocity commands (wheel rad/s)      -> TinkerStepperCommand

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "rclcpp/rclcpp.hpp"

using cmeresearch_msgs::msg::TinkerStepperCommand;
using cmeresearch_msgs::msg::TinkerStepperFeedback;
using namespace std::chrono_literals;

namespace
{

const std::vector<std::string> kWheels = {"front_left", "front_right", "rear_left", "rear_right"};

constexpr double kSteps = 200.0;
constexpr double kMicro = 8.0;
constexpr double kGear = 2.0;
// One wheel revolution per second == 200 * 8 * 2 microsteps/s on the motor.
constexpr double kMicrostepsPerWheelRev = kSteps * kMicro * kGear;

std::string make_urdf(const std::map<std::string, std::string> & hw_params)
{
  std::string urdf =
    R"(<?xml version="1.0"?><robot name="cmexa_test"><link name="base_link"/>)";
  for (const auto & w : kWheels) {
    urdf += "<link name=\"" + w + "_wheel\"/>"
      "<joint name=\"" + w + "_wheel_joint\" type=\"continuous\">"
      "<parent link=\"base_link\"/><child link=\"" + w + "_wheel\"/><axis xyz=\"0 1 0\"/>"
      "</joint>";
  }
  urdf += R"(<ros2_control name="CmexaBase" type="system"><hardware>)"
    R"(<plugin>cmexa_base/CmexaBaseBotSystemHardware</plugin>)";
  for (const auto & [key, value] : hw_params) {
    urdf += "<param name=\"" + key + "\">" + value + "</param>";
  }
  urdf += "</hardware>";
  for (const auto & w : kWheels) {
    urdf += "<joint name=\"" + w + "_wheel_joint\">"
      "<command_interface name=\"velocity\"/>"
      "<state_interface name=\"position\"/><state_interface name=\"velocity\"/></joint>";
  }
  urdf += "</ros2_control></robot>";
  return urdf;
}

std::map<std::string, std::string> default_params()
{
  std::map<std::string, std::string> p = {
    {"example_param_hw_start_duration_sec", "0"},
    {"example_param_hw_stop_duration_sec", "0"},
    {"gear_ratio", "2.0"},
    {"wheel_radius", "0.05"},
    {"wheel_separation_x", "0.3"},
    {"wheel_separation_y", "0.3"},
    {"steps_per_revolution", "200"},
    {"step_resolution", "8"},
    // 3200 microsteps/s == 1 wheel rev/s == 2*pi wheel rad/s ceiling.
    {"max_step_vel", "3200"},
  };
  for (const auto & w : kWheels) {
    p[w + "_cmd_topic"] = "/cmexa_test/" + w + "/cmd_vel";
    p[w + "_feedback_topic"] = "/cmexa_test/" + w + "/feedback";
  }
  return p;
}

class CmexaBaseSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override {start(default_params());}

  void start(const std::map<std::string, std::string> & hw_params)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<rclcpp::Node>("cmexa_base_test");
    executor_->add_node(node_);

    for (const auto & w : kWheels) {
      feedback_pubs_[w] = node_->create_publisher<TinkerStepperFeedback>(
        "/cmexa_test/" + w + "/feedback", 10);
      cmd_subs_[w] = node_->create_subscription<TinkerStepperCommand>(
        "/cmexa_test/" + w + "/cmd_vel", 10,
        [this, w](TinkerStepperCommand::SharedPtr msg) {last_cmd_[w] = msg;});
    }

    hardware_interface::ResourceManagerParams params;
    params.robot_description = make_urdf(hw_params);
    params.clock = node_->get_clock();
    params.logger = node_->get_logger();
    params.executor = executor_;
    params.activate_all = true;
    rm_ = std::make_unique<hardware_interface::ResourceManager>(params, true);
  }

  void TearDown() override
  {
    rm_.reset();
    executor_.reset();
    node_.reset();
  }

  /// Spin until pred() is true or the timeout elapses.
  bool spin_until(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 5s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
      if (pred()) {
        return true;
      }
    }
    return false;
  }

  double state(const std::string & wheel, const std::string & iface)
  {
    auto loaned = rm_->claim_state_interface(wheel + "_wheel_joint/" + iface);
    return loaned.get_optional<double>().value();
  }

  /// Publish feedback (microsteps/s) on every wheel until read() reflects it.
  void feed(const std::map<std::string, double> & steps_s)
  {
    const auto period = rclcpp::Duration(100ms);
    ASSERT_TRUE(
      spin_until(
        [&]() {
          for (const auto & [w, v] : steps_s) {
            TinkerStepperFeedback msg;
            msg.current_velocity = static_cast<float>(v);
            feedback_pubs_[w]->publish(msg);
          }
          executor_->spin_some(10ms);
          rm_->read(node_->now(), period);
          for (const auto & [w, v] : steps_s) {
            const double want = v * 2.0 * M_PI / kMicrostepsPerWheelRev;
            if (std::abs(state(w, "velocity") - want) > 1e-6) {
              return false;
            }
          }
          return true;
        })) << "feedback never reached the joint states";
  }

  /// Set joint velocity commands, call write() until every wheel's stepper
  /// command has been received; returns the received velocities.
  std::map<std::string, double> command(const std::map<std::string, double> & wheel_rad_s)
  {
    std::vector<hardware_interface::LoanedCommandInterface> loans;
    for (const auto & [w, v] : wheel_rad_s) {
      loans.push_back(rm_->claim_command_interface(w + "_wheel_joint/velocity"));
      EXPECT_TRUE(loans.back().set_value(v));
    }
    last_cmd_.clear();
    const bool ok = spin_until(
      [&]() {
        rm_->write(node_->now(), rclcpp::Duration(10ms));
        executor_->spin_some(10ms);
        return last_cmd_.size() == kWheels.size();
      });
    EXPECT_TRUE(ok) << "stepper commands not received";
    std::map<std::string, double> out;
    for (const auto & [w, msg] : last_cmd_) {
      out[w] = msg->velocity;
    }
    return out;
  }

  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;
  std::map<std::string, rclcpp::Publisher<TinkerStepperFeedback>::SharedPtr> feedback_pubs_;
  std::map<std::string, rclcpp::Subscription<TinkerStepperCommand>::SharedPtr> cmd_subs_;
  std::map<std::string, TinkerStepperCommand::SharedPtr> last_cmd_;
};

}  // namespace

TEST_F(CmexaBaseSystemTest, ExportsFourWheelInterfaces)
{
  for (const auto & w : kWheels) {
    EXPECT_TRUE(rm_->command_interface_exists(w + "_wheel_joint/velocity")) << w;
    EXPECT_TRUE(rm_->state_interface_exists(w + "_wheel_joint/velocity")) << w;
    EXPECT_TRUE(rm_->state_interface_exists(w + "_wheel_joint/position")) << w;
  }
}

TEST_F(CmexaBaseSystemTest, FeedbackIsScaledToWheelRadPerSecond)
{
  // One wheel revolution per second forward on every wheel.
  feed({{"front_left", kMicrostepsPerWheelRev}, {"front_right", kMicrostepsPerWheelRev},
    {"rear_left", kMicrostepsPerWheelRev}, {"rear_right", kMicrostepsPerWheelRev}});
  for (const auto & w : kWheels) {
    EXPECT_NEAR(state(w, "velocity"), 2.0 * M_PI, 1e-6) << w;
  }
}

TEST_F(CmexaBaseSystemTest, NegativeFeedbackGivesNegativeWheelVelocity)
{
  // Regression guard for the odometry bug: reversing wheels must not read as forward.
  feed({{"front_left", -kMicrostepsPerWheelRev / 2}, {"front_right", kMicrostepsPerWheelRev / 2},
    {"rear_left", kMicrostepsPerWheelRev / 2}, {"rear_right", -kMicrostepsPerWheelRev / 2}});
  EXPECT_NEAR(state("front_left", "velocity"), -M_PI, 1e-6);
  EXPECT_NEAR(state("front_right", "velocity"), M_PI, 1e-6);
  EXPECT_NEAR(state("rear_left", "velocity"), M_PI, 1e-6);
  EXPECT_NEAR(state("rear_right", "velocity"), -M_PI, 1e-6);
}

TEST_F(CmexaBaseSystemTest, PositionIntegratesVelocity)
{
  feed({{"front_left", -kMicrostepsPerWheelRev}, {"front_right", 0.0},
    {"rear_left", 0.0}, {"rear_right", 0.0}});
  const double before = state("front_left", "position");
  for (int i = 0; i < 10; ++i) {
    rm_->read(node_->now(), rclcpp::Duration(100ms));
  }
  // 10 * 0.1 s at -2*pi rad/s.
  EXPECT_NEAR(state("front_left", "position") - before, -2.0 * M_PI, 1e-6);
  EXPECT_NEAR(state("front_right", "position"), 0.0, 1e-9);
}

TEST_F(CmexaBaseSystemTest, CommandsArePassedThroughInWheelRadPerSecond)
{
  const auto out = command(
    {{"front_left", 1.0}, {"front_right", -0.5}, {"rear_left", 0.0}, {"rear_right", -2.0}});
  EXPECT_NEAR(out.at("front_left"), 1.0, 1e-6);
  EXPECT_NEAR(out.at("front_right"), -0.5, 1e-6);
  EXPECT_NEAR(out.at("rear_left"), 0.0, 1e-6);
  EXPECT_NEAR(out.at("rear_right"), -2.0, 1e-6);
  EXPECT_EQ(last_cmd_.at("front_left")->header.frame_id, "front_left_wheel_joint");
}

TEST_F(CmexaBaseSystemTest, CommandsAboveCeilingAreScaledTogether)
{
  // Ceiling is 2*pi (max_step_vel 3200). Fastest wheel 4*pi -> scale 0.5 for all,
  // preserving the direction of the mecanum motion vector.
  const auto out = command(
    {{"front_left", 4.0 * M_PI}, {"front_right", -2.0 * M_PI}, {"rear_left", M_PI},
      {"rear_right", 0.0}});
  EXPECT_NEAR(out.at("front_left"), 2.0 * M_PI, 1e-5);
  EXPECT_NEAR(out.at("front_right"), -M_PI, 1e-5);
  EXPECT_NEAR(out.at("rear_left"), M_PI / 2.0, 1e-5);
  EXPECT_NEAR(out.at("rear_right"), 0.0, 1e-6);
}

TEST_F(CmexaBaseSystemTest, CommandsBelowCeilingAreUntouched)
{
  const auto out = command(
    {{"front_left", 2.0 * M_PI - 0.01}, {"front_right", 0.1}, {"rear_left", -0.1},
      {"rear_right", 0.0}});
  EXPECT_NEAR(out.at("front_left"), 2.0 * M_PI - 0.01, 1e-5);
  EXPECT_NEAR(out.at("front_right"), 0.1, 1e-6);
}

class CmexaBaseInitTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(CmexaBaseInitTest, MissingRequiredParameterFailsInit)
{
  auto node = std::make_shared<rclcpp::Node>("cmexa_base_init_test");
  auto params = default_params();
  params.erase("gear_ratio");

  hardware_interface::ResourceManagerParams rm_params;
  rm_params.robot_description = make_urdf(params);
  rm_params.clock = node->get_clock();
  rm_params.logger = node->get_logger();
  rm_params.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  hardware_interface::ResourceManager rm(rm_params, false);
  bool loaded = false;
  try {
    loaded = rm.load_and_initialize_components(rm_params);
  } catch (const std::exception &) {
    loaded = false;
  }
  EXPECT_FALSE(loaded && rm.command_interface_exists("front_left_wheel_joint/velocity"));
}
