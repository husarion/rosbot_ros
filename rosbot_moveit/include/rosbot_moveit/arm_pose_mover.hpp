// Copyright (c) 2024 Husarion Sp. z o.o.
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

#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rosbot_moveit {

// Workspace cube half-edge (m) for MGI::setWorkspace; bounds planner sampling
// and silences ValidateWorkspaceBounds substituting a 1e12 default.
constexpr double ARM_WORKSPACE_HALF_EDGE = 0.5;

struct NamedTarget {
  std::string group_name;
  std::string pose_name;
  std::optional<double> velocity_scaling;
  std::optional<double> acceleration_scaling;
  int max_attempts = 1;
};

class ArmPoseMover {
public:
  explicit ArmPoseMover(rclcpp::Node::SharedPtr node);
  ~ArmPoseMover();

  ArmPoseMover(const ArmPoseMover &) = delete;
  ArmPoseMover &operator=(const ArmPoseMover &) = delete;

  bool WaitForMoveGroup(std::chrono::seconds timeout);
  // Waits for each `<group>_controller/follow_joint_trajectory` action server
  // to become available. Without this MGI::move() races the controller_manager
  // spawner on boot and reports spurious "Action client not connected" errors.
  bool WaitForControllers(const std::vector<std::string> &group_names,
                          std::chrono::seconds timeout);
  bool MoveToTargets(const std::vector<NamedTarget> &sequence);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

} // namespace rosbot_moveit
