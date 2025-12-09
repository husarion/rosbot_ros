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

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>

using MGI = moveit::planning_interface::MoveGroupInterface;

bool wait_for_moveit_server(const std::shared_ptr<rclcpp::Node> &node) {
  auto client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      node, "move_action");
  if (!client->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(node->get_logger(), "MoveGroup server not available!");
    return false;
  }
  return true;
}

bool move_to_named_target(MGI &group, const std::string &target_name,
                          const std::shared_ptr<rclcpp::Node> &node,
                          int max_attempts = 1) {
  group.setNamedTarget(target_name);
  bool success = false;
  for (int attempt = 0; attempt < max_attempts && !success; ++attempt) {
    auto result = group.move();
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      success = true;
    } else {
      RCLCPP_WARN_STREAM(node->get_logger(),
                         "Failed to move to '"
                             << target_name << "' pose (attempt "
                             << (attempt + 1) << "/" << max_attempts
                             << "), retrying...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  return success;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("move_to_home_pose", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  if (!wait_for_moveit_server(node)) {
    rclcpp::shutdown();
    return 1;
  }

  MGI::Options manip_opts("manipulator", "robot_description",
                          node->get_namespace());
  MGI manipulator_group(node, manip_opts);
  manipulator_group.setMaxVelocityScalingFactor(0.2);
  manipulator_group.setMaxAccelerationScalingFactor(0.1);

  if (!move_to_named_target(manipulator_group, "Home", node, 3)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to move to 'Home' pose after retries.");
    rclcpp::shutdown();
    return 1;
  }

  MGI::Options gripper_opts("gripper", "robot_description",
                            node->get_namespace());
  MGI gripper_group(node, gripper_opts);
  gripper_group.setMaxVelocityScalingFactor(0.6);
  gripper_group.setMaxAccelerationScalingFactor(0.6);
  move_to_named_target(gripper_group, "Open", node);

  rclcpp::shutdown();
  return 0;
}
