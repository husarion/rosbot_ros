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

#include "rosbot_hardware_interfaces/rosbot_imu_sensor.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rosbot_hardware_interfaces {
CallbackReturn RosbotImuSensor::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Initializing");

  if (hardware_interface::SensorInterface::on_init(params) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

  connection_timeout_ms_ =
      std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ =
      std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

  node_ = get_node();
  auto ns = node_->get_namespace();
  RCLCPP_INFO(node_->get_logger(), "Creating node with namespace: %s", ns);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotImuSensor::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotImuSensor::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotImuSensor::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Activating");

  for (auto i = 0u; i < imu_sensor_state_.size(); i++) {
    imu_sensor_state_[i] = 0.0;
  }

  imu_subscriber_ = node_->create_subscription<Imu>(
      "_imu/data", rclcpp::SensorDataQoS(),
      std::bind(&RosbotImuSensor::imu_cb, this, std::placeholders::_1));

  std::shared_ptr<Imu> imu_msg;
  for (uint wait_time = 0; wait_time <= connection_timeout_ms_;
       wait_time += connection_check_period_ms_) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(
          rclcpp::get_logger("RosbotImuSensor"),
          "ROS shutdown signal detected while waiting for IMU messages.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
        rclcpp::get_logger("RosbotImuSensor"), *node_->get_clock(), 5000,
        "Feedback message from imu wasn't received yet");
    received_imu_msg_ptr_.get([&](const auto &msg) { imu_msg = msg; });
    if (imu_msg) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Subscriber and publisher are now active.");
      return CallbackReturn::SUCCESS;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
  }

  RCLCPP_FATAL(
      node_->get_logger(),
      "Activation failed, timeout reached while waiting for feedback from imu");
  return CallbackReturn::ERROR;
}

CallbackReturn RosbotImuSensor::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Deactivating");
  cleanup_node();
  received_imu_msg_ptr_.set(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotImuSensor::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotImuSensor::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotImuSensor"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RosbotImuSensor::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name,
        &imu_sensor_state_[i]));
  }

  return state_interfaces;
}

void RosbotImuSensor::cleanup_node() { imu_subscriber_.reset(); }

void RosbotImuSensor::imu_cb(const std::shared_ptr<Imu> msg) {
  RCLCPP_DEBUG(node_->get_logger(), "Received imu message");
  received_imu_msg_ptr_.set([&](auto &msg_ref) { msg_ref = std::move(msg); });
}

return_type RosbotImuSensor::read(const rclcpp::Time &,
                                  const rclcpp::Duration &) {
  std::shared_ptr<Imu> imu_msg;
  received_imu_msg_ptr_.get([&](const auto &msg) { imu_msg = msg; });

  RCLCPP_DEBUG(rclcpp::get_logger("RosbotImuSensor"), "Reading imu state");

  if (!imu_msg) {
    RCLCPP_ERROR(rclcpp::get_logger("RosbotImuSensor"),
                 "Imu message wasn't received");
    return return_type::ERROR;
  }

  imu_sensor_state_[0] = imu_msg->orientation.x;
  imu_sensor_state_[1] = imu_msg->orientation.y;
  imu_sensor_state_[2] = imu_msg->orientation.z;
  imu_sensor_state_[3] = imu_msg->orientation.w;
  imu_sensor_state_[4] = imu_msg->angular_velocity.x;
  imu_sensor_state_[5] = imu_msg->angular_velocity.y;
  imu_sensor_state_[6] = imu_msg->angular_velocity.z;
  imu_sensor_state_[7] = imu_msg->linear_acceleration.x;
  imu_sensor_state_[8] = imu_msg->linear_acceleration.y;
  imu_sensor_state_[9] = imu_msg->linear_acceleration.z;

  return return_type::OK;
}

} // namespace rosbot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot_hardware_interfaces::RosbotImuSensor,
                       hardware_interface::SensorInterface)
