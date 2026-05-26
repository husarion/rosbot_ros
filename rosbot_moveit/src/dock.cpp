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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("move_to_dock_pose", options);

  rosbot_moveit::ArmPoseMover mover(node);
  if (!mover.WaitForMoveGroup(std::chrono::seconds(5))) {
    rclcpp::shutdown();
    return 1;
  }
  if (!mover.WaitForControllers({"manipulator", "gripper"},
                                std::chrono::seconds(15))) {
    rclcpp::shutdown();
    return 1;
  }

  const bool ok = mover.MoveToTargets({
      {"gripper", "Close", std::nullopt, std::nullopt, 3},
      {"manipulator", "Dock", 0.2, 0.1, 1},
  });

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
