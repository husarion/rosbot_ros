// Copyright 2024 Husarion sp. z o.o.
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

#include "rosbot_hardware_interfaces/rosbot_system.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rosbot_hardware_interfaces {
CallbackReturn
RosbotSystem::on_init(const hardware_interface::HardwareInfo &hardware_info) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RosbotSystem"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  for (auto &j : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Joint '%s' found",
                j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }

  connection_timeout_ms_ =
      std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ =
      std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

  std::string velocity_command_joint_order_raw =
      info_.hardware_parameters["velocity_command_joint_order"];
  // remove whitespaces
  velocity_command_joint_order_raw.erase(
      std::remove_if(
          velocity_command_joint_order_raw.begin(),
          velocity_command_joint_order_raw.end(),
          [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
      velocity_command_joint_order_raw.end());
  std::stringstream velocity_command_joint_order_stream(
      velocity_command_joint_order_raw);
  std::string joint_name;
  while (getline(velocity_command_joint_order_stream, joint_name, ',')) {
    velocity_command_joint_order_.push_back(joint_name);
  }

  if (velocity_command_joint_order_.size() != info_.joints.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                 "Joint order size is invalid");
    return CallbackReturn::ERROR;
  }

  for (auto &j : info_.joints) {
    if (std::find(velocity_command_joint_order_.begin(),
                  velocity_command_joint_order_.end(),
                  j.name) == velocity_command_joint_order_.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"),
                   "Joint '%s' missing from velocity command joint order",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("rosbot_system_node");
  executor_.add_node(node_);
  executor_thread_ = std::make_unique<std::thread>(
      std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Activating");

  for (const auto &x : pos_state_) {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    vel_commands_[x.first] = 0.0;
  }

  motor_command_publisher_ = node_->create_publisher<Float32MultiArray>(
      "~/motors_cmd", rclcpp::SensorDataQoS());
  realtime_motor_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(
          motor_command_publisher_);

  motor_state_subscriber_ = node_->create_subscription<JointState>(
      "~/motors_response", rclcpp::SensorDataQoS(),
      std::bind(&RosbotSystem::motor_state_cb, this, std::placeholders::_1));

  std::shared_ptr<JointState> motor_state;
  for (uint wait_time = 0; wait_time <= connection_timeout_ms_;
       wait_time += connection_check_period_ms_) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(rclcpp::get_logger("RosbotSystem"),
                  "ROS shutdown signal detected while waiting for motor "
                  "feedback messages.");
      return CallbackReturn::ERROR;
    }

    RCLCPP_WARN_SKIPFIRST_THROTTLE(
        rclcpp::get_logger("RosbotSystem"), *node_->get_clock(), 5000,
        "Feedback message from motors wasn't received yet");
    received_motor_state_msg_ptr_.get(
        [&](const auto &msg) { motor_state = msg; });

    if (motor_state) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Subscriber and publisher are now active.");
      return CallbackReturn::SUCCESS;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
  }

  RCLCPP_FATAL(node_->get_logger(), "Activation failed, timeout reached while "
                                    "waiting for feedback from motors");
  return CallbackReturn::ERROR;
}

CallbackReturn RosbotSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Deactivating");
  cleanup_node();
  received_motor_state_msg_ptr_.set(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RosbotSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                       &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                       &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> RosbotSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

void RosbotSystem::cleanup_node() {
  motor_state_subscriber_.reset();
  realtime_motor_command_publisher_.reset();
  motor_command_publisher_.reset();
}

void RosbotSystem::motor_state_cb(const std::shared_ptr<JointState> msg) {
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");
  received_motor_state_msg_ptr_.set(
      [&](auto &msg_ref) { msg_ref = std::move(msg); });
}

return_type RosbotSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(
      [&](const auto &msg) { motor_state = msg; });

  RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"), "Reading motors state");

  if (!motor_state) {
    RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"),
                 "Feedback message from motors wasn't received");
    return return_type::ERROR;
  }

  for (auto i = 0u; i < motor_state->name.size(); i++) {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
        vel_state_.find(motor_state->name[i]) == vel_state_.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"),
                   "Position or velocity feedback not found for joint %s",
                   motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i];

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"),
                 "Position feedback: %f, velocity feedback: %f",
                 pos_state_[motor_state->name[i]],
                 vel_state_[motor_state->name[i]]);
  }
  return return_type::OK;
}

return_type RosbotSystem::write(const rclcpp::Time &,
                                const rclcpp::Duration &) {
  if (realtime_motor_command_publisher_->trylock()) {
    auto &motor_command = realtime_motor_command_publisher_->msg_;
    motor_command.data.clear();

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"),
                 "Wrtiting motors cmd message");

    for (auto const &joint : velocity_command_joint_order_) {
      motor_command.data.push_back(vel_commands_[joint]);
    }

    realtime_motor_command_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}

} // namespace rosbot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot_hardware_interfaces::RosbotSystem,
                       hardware_interface::SystemInterface)
