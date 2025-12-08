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

#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

namespace rosbot_joy::servo {

enum CommandType {
  NONE = -1,
  JOINT_JOG = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG,
  TWIST = moveit_msgs::srv::ServoCommandType::Request::TWIST,
  POSE = moveit_msgs::srv::ServoCommandType::Request::POSE,
};

enum Axis {
  LEFT_STICK_HORIZONTAL = 0,
  LEFT_STICK_VERTICAL = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_HORIZONTAL = 3,
  RIGHT_STICK_VERTICAL = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_HORIZONTAL = 6,
  D_PAD_VERTICAL = 7
};

enum Button {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  BACK = 6,
  START = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

const std::string TWIST_TOPIC = "servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "servo_node/delta_joint_cmds";
const std::string GRIPPER_ACTION = "gripper_controller/gripper_cmd";

const size_t ROS_QUEUE_SIZE = 10;
const std::string EE_FRAME_ID = "end_effector_link";
const double DEAD_MAN_SWITCH_THRESHOLD = -0.3;
const double GRIPPER_MIN_POSE = -0.009;
const double GRIPPER_MAX_POSE = 0.015;
const double MAX_CMD_TYPE_REQ_PERIOD = 0.5;
// const std::vector<double> GRIPPER_MAX_EFFORT = { 10.0 };
const double GRIPPER_MAX_EFFORT = 10.0;
const std::vector<std::string> GRIPPER_JOINT_NAME = {"gripper_left_joint"};
const std::vector<std::string> JOINT_NAMES = {"joint1", "joint2", "joint3",
                                              "joint4"};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a
// controller
class Joy2Servo : public rclcpp::Node {
public:
  Joy2Servo();
  void InitializeMoveGroup();

private:
  void ChangeCommandType(const CommandType cmd_type);
  void ChangeCommandTypeCallback(
      const rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture
          future);
  void ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg);
  void ConvertAndPublishJoint(const sensor_msgs::msg::Joy::SharedPtr msg);
  void ConvertAndPublishTwist(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg);
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void MoveToDockPose();
  void MoveToHomePose();
  void UpdateReqCommand(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr
      switch_cmd_type_srv_;
  moveit::planning_interface::MoveGroupInterfacePtr gripper_group_;
  moveit::planning_interface::MoveGroupInterfacePtr manipulator_group_;

  CommandType req_cmd_type_ = CommandType::JOINT_JOG;
  CommandType cmd_type_ = CommandType::NONE;
  double joint_vel_cmd_; // TODO: Add scaler
  double gripper_position_;
  std::mutex joy_mutex_; // Add this to your class
};

} // namespace rosbot_joy::servo
