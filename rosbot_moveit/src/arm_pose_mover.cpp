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

#include <rosbot_moveit/arm_pose_mover.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rosbot_moveit {

using MGI = moveit::planning_interface::MoveGroupInterface;

ArmPoseMover::ArmPoseMover(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {
  executor_.add_node(node_);
  spinner_ = std::thread([this]() { executor_.spin(); });
}

ArmPoseMover::~ArmPoseMover() {
  executor_.cancel();
  if (spinner_.joinable()) {
    spinner_.join();
  }
}

bool ArmPoseMover::WaitForMoveGroup(std::chrono::seconds timeout) {
  auto client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      node_, "move_action");
  if (!client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroup action server not available");
    return false;
  }
  return true;
}

bool ArmPoseMover::WaitForControllers(
    const std::vector<std::string> &group_names, std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (const auto &group : group_names) {
    const std::string action_name =
        group + "_controller/follow_joint_trajectory";
    auto client = rclcpp_action::create_client<
        control_msgs::action::FollowJointTrajectory>(node_, action_name);
    const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining <= std::chrono::nanoseconds(0) ||
        !client->wait_for_action_server(remaining)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Action server '"
                                                   << action_name
                                                   << "' not available");
      return false;
    }
  }
  return true;
}

bool ArmPoseMover::MoveToTargets(const std::vector<NamedTarget> &sequence) {
  for (const auto &target : sequence) {
    MGI::Options opts(target.group_name, MGI::ROBOT_DESCRIPTION,
                      node_->get_namespace());
    MGI group(node_, opts);
    group.setWorkspace(-ARM_WORKSPACE_HALF_EDGE, -ARM_WORKSPACE_HALF_EDGE,
                       -ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE,
                       ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE);
    if (target.velocity_scaling) {
      group.setMaxVelocityScalingFactor(*target.velocity_scaling);
    }
    if (target.acceleration_scaling) {
      group.setMaxAccelerationScalingFactor(*target.acceleration_scaling);
    }
    group.setNamedTarget(target.pose_name);

    bool success = false;
    for (int attempt = 0; attempt < target.max_attempts && !success;
         ++attempt) {
      if (group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
        success = true;
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Failed to move " << target.group_name << " to '"
                                             << target.pose_name
                                             << "' (attempt " << (attempt + 1)
                                             << "/" << target.max_attempts
                                             << ")");
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (!success) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "Failed to move " << target.group_name << " to '"
                                            << target.pose_name << "' after "
                                            << target.max_attempts
                                            << " attempts");
      return false;
    }
  }
  return true;
}

} // namespace rosbot_moveit
