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
  auto node = rclcpp::Node::make_shared("move_to_home_pose", options);

  rosbot_moveit::ArmPoseMover mover(node);
  // Timeouts cover the full bringup from `ros2 launch` start: move_group
  // finishes RDF/SRDF/kinematics load in ~5s after manipulator.yaml include,
  // controllers wait for ros2_control_node which itself depends on the
  // Dynamixel chain initialisation (~10s for 5 servos at 1 Mbps).
  if (!mover.WaitForMoveGroup(std::chrono::seconds(30))) {
    rclcpp::shutdown();
    return 1;
  }
  if (!mover.WaitForControllers({"manipulator", "gripper"},
                                std::chrono::seconds(60))) {
    rclcpp::shutdown();
    return 1;
  }

  const bool ok = mover.MoveToTargets({
      {"manipulator", "Home", 0.2, 0.1, 3},
      {"gripper", "Open", 0.6, 0.6, 3},
  });

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
