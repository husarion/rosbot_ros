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

// Half-edge of the workspace cube (in metres) passed to
// MoveGroupInterface::setWorkspace by every node that talks to move_group in
// this package. OpenManipulator-X reach ≈ 0.4 m; 0.5 m gives a comfortable
// margin while still bounding planner sampling. Without setWorkspace,
// MotionPlanRequest.workspace_parameters arrive zero and
// ValidateWorkspaceBounds warns + substitutes a huge (1e12) default.
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
  bool MoveToTargets(const std::vector<NamedTarget> &sequence);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

} // namespace rosbot_moveit
