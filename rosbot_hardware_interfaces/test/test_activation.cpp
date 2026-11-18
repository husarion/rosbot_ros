// Copyright 2026 Husarion sp. z o.o.
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

// on_activate runs inside a controller_manager executor callback. These tests
// hand the component an executor that nobody spins, which is what a
// single-threaded CM executor looks like from inside on_activate.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rosbot_hardware_interfaces/rosbot_imu_sensor.hpp"
#include "rosbot_hardware_interfaces/rosbot_system.hpp"

using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;
using rosbot_hardware_interfaces::CallbackReturn;

namespace {

const char *const kJoints[] = {"fl_wheel_joint", "fr_wheel_joint",
                               "rl_wheel_joint", "rr_wheel_joint"};

InterfaceInfo Interface(const std::string &name) {
  InterfaceInfo info;
  info.name = name;
  return info;
}

void SetConnectionParams(HardwareInfo &info) {
  info.hardware_parameters["connection_timeout_ms"] = "300";
  info.hardware_parameters["connection_check_period_ms"] = "20";
}

HardwareInfo SystemInfo() {
  HardwareInfo info;
  info.name = "test_rosbot_system";
  info.type = "system";
  info.hardware_plugin_name = "rosbot_hardware_interfaces/RosbotSystem";
  SetConnectionParams(info);
  std::string order;
  for (const auto *joint : kJoints) {
    ComponentInfo c;
    c.name = joint;
    c.type = "joint";
    c.command_interfaces.push_back(
        Interface(hardware_interface::HW_IF_VELOCITY));
    c.state_interfaces.push_back(Interface(hardware_interface::HW_IF_POSITION));
    c.state_interfaces.push_back(Interface(hardware_interface::HW_IF_VELOCITY));
    info.joints.push_back(c);
    order += (order.empty() ? "" : ",") + std::string(joint);
  }
  info.hardware_parameters["velocity_command_joint_order"] = order;
  return info;
}

HardwareInfo ImuInfo() {
  HardwareInfo info;
  info.name = "test_rosbot_imu";
  info.type = "sensor";
  info.hardware_plugin_name = "rosbot_hardware_interfaces/RosbotImuSensor";
  SetConnectionParams(info);
  ComponentInfo c;
  c.name = "imu";
  c.type = "sensor";
  for (const auto *name :
       {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y",
        "linear_acceleration.z"}) {
    c.state_interfaces.push_back(Interface(name));
  }
  info.sensors.push_back(c);
  return info;
}

template <typename MsgT> class FeedbackPublisher {
public:
  FeedbackPublisher(const std::string &topic, MsgT msg)
      : node_(std::make_shared<rclcpp::Node>("test_feedback_publisher")),
        pub_(node_->create_publisher<MsgT>(topic, rclcpp::SensorDataQoS())),
        thread_([this, msg]() {
          while (running_) {
            pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }) {}

  ~FeedbackPublisher() {
    running_ = false;
    thread_.join();
  }

private:
  std::atomic<bool> running_{true};
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  std::thread thread_;
};

sensor_msgs::msg::JointState MotorFeedback() {
  sensor_msgs::msg::JointState msg;
  for (const auto *joint : kJoints) {
    msg.name.push_back(joint);
    msg.position.push_back(0.0);
    msg.velocity.push_back(0.0);
  }
  return msg;
}

template <typename ComponentT>
CallbackReturn InitAndActivate(ComponentT &component, const HardwareInfo &info,
                               const rclcpp::Executor::SharedPtr &executor) {
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = info;
  params.clock = std::make_shared<rclcpp::Clock>();
  params.executor = executor;
  if (component.init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  const rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  return component.on_activate(inactive);
}

class ActivationWithoutSpinningExecutor : public ::testing::Test {
protected:
  rclcpp::Executor::SharedPtr executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
};

} // namespace

TEST_F(ActivationWithoutSpinningExecutor, SystemActivatesOnFeedback) {
  FeedbackPublisher<sensor_msgs::msg::JointState> feedback("_motors/feedback",
                                                           MotorFeedback());
  rosbot_hardware_interfaces::RosbotSystem system;
  EXPECT_EQ(InitAndActivate(system, SystemInfo(), executor_),
            CallbackReturn::SUCCESS);
}

TEST_F(ActivationWithoutSpinningExecutor, SystemFailsWithoutFeedback) {
  rosbot_hardware_interfaces::RosbotSystem system;
  EXPECT_EQ(InitAndActivate(system, SystemInfo(), executor_),
            CallbackReturn::ERROR);
}

TEST_F(ActivationWithoutSpinningExecutor, ImuActivatesOnFeedback) {
  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  FeedbackPublisher<sensor_msgs::msg::Imu> feedback("_imu/data", imu);
  rosbot_hardware_interfaces::RosbotImuSensor sensor;
  EXPECT_EQ(InitAndActivate(sensor, ImuInfo(), executor_),
            CallbackReturn::SUCCESS);
}

TEST_F(ActivationWithoutSpinningExecutor, ImuFailsWithoutFeedback) {
  rosbot_hardware_interfaces::RosbotImuSensor sensor;
  EXPECT_EQ(InitAndActivate(sensor, ImuInfo(), executor_),
            CallbackReturn::ERROR);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
