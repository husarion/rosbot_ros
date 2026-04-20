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

#ifndef ROSBOT_HARDWARE_INTERFACES__ROSBOT_IMU_SENSOR_HPP_
#define ROSBOT_HARDWARE_INTERFACES__ROSBOT_IMU_SENSOR_HPP_

#include "rosbot_hardware_interfaces/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "sensor_msgs/msg/imu.hpp"

namespace rosbot_hardware_interfaces {
using return_type = hardware_interface::return_type;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using Imu = sensor_msgs::msg::Imu;

class RosbotImuSensor : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RosbotImuSensor)

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  ROSBOT_HARDWARE_INTERFACES_PUBLIC
  return_type read(const rclcpp::Time &time,
                   const rclcpp::Duration &period) override;

protected:
  void cleanup_node();

  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<Imu>>
      received_imu_msg_ptr_{nullptr};

  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_ = nullptr;

  std::vector<double> imu_sensor_state_;

  bool subscriber_is_active_ = false;

  std::shared_ptr<rclcpp::Node> node_;

  void imu_cb(const std::shared_ptr<Imu> msg);
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;

  uint connection_check_period_ms_;
  uint connection_timeout_ms_;
};

} // namespace rosbot_hardware_interfaces

#endif // ROSBOT_HARDWARE_INTERFACES__ROSBOT_IMU_SENSOR_HPP_
