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

#ifndef ROSBOT_JOY__SERVO__MANIPULATION_CONTROLLER_HPP_
#define ROSBOT_JOY__SERVO__MANIPULATION_CONTROLLER_HPP_

#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rosbot_joy/servo/joy_control.hpp>

namespace rosbot_joy::servo {

using MGI = moveit::planning_interface::MoveGroupInterface;
using MGI_Ptr = moveit::planning_interface::MoveGroupInterfacePtr;

class ManipulationController {
public:
  virtual ~ManipulationController() = default;

  /**
   * @brief Checks if button/axis was activated, if so send a command
   *
   * @returns true if button/axis was activated
   */
  virtual bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
  virtual void Stop() = 0;

protected:
  bool CheckIfPressed(
      const sensor_msgs::msg::Joy::SharedPtr msg,
      const std::map<std::string, std::unique_ptr<JoyControl>> &controls);

  std::vector<double> CalculateCommand(
      const sensor_msgs::msg::Joy::SharedPtr msg,
      const std::vector<std::string> &cmd_names,
      const std::map<std::string, std::unique_ptr<JoyControl>> &controls);
};

class JointController : public ManipulationController {
public:
  JointController(const rclcpp::Node::SharedPtr &node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override;

private:
  void ParseParameters(const rclcpp::Node::SharedPtr &node);

  void SendJointCommand(const std::vector<double> &cmds,
                        const builtin_interfaces::msg::Time &timestamp);

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>>
      manipulator_joint_controls_;

  std::vector<std::string> joint_names_;
};

class CartesianController : public ManipulationController {
public:
  CartesianController(const rclcpp::Node::SharedPtr &node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override;

private:
  void ParseParameters(const rclcpp::Node::SharedPtr &node);

  void SendCartesianCommand(const std::vector<double> &cmds,
                            const builtin_interfaces::msg::Time &timestamp);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>>
      manipulator_cartesian_controls_;

  std::vector<std::string> cartesian_control_names_;
  std::vector<std::string> cartesian_cmd_names_;
  std::string cartesian_control_reference_frame_;
};

class ManipulatorMoveGroupController : public ManipulationController {
public:
  ManipulatorMoveGroupController(const rclcpp::Node::SharedPtr &node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override {}

private:
  void ParseParameters(const rclcpp::Node::SharedPtr &node);
  void ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg);
  void MoveToDockPose();
  void MoveToHomePose();

  MGI_Ptr gripper_group_;
  MGI_Ptr manipulator_group_;

  std::unique_ptr<JoyControl> gripper_cmd_;
  std::unique_ptr<JoyControl> gripper_trigger_;
  std::unique_ptr<JoyControl> dock_manipulator_;
  std::unique_ptr<JoyControl> home_manipulator_;

  // action of this controller should be triggered only once per button press
  //  and require releasing button before executing again
  std::string joint_name_gripper_ = "gripper_finger_joint";
  bool is_action_executing_ = false;
  double gripper_position_ = 0.0;
  double gripper_min_pose_;
  double gripper_max_pose_;
};

} // namespace rosbot_joy::servo

#endif // ROSBOT_JOY__SERVO__MANIPULATION_CONTROLLER_HPP_
