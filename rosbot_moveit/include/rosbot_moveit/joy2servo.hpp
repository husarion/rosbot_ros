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

#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rosbot_moveit {

// Both modes publish JointJog (skips servo's singularity guard, which is
// unconditional on POSE/TWIST and rank-deficient on 4-DoF, see
// moveit_msgs#185).
//   Cartesian (Y): sticks -> EE velocity -> local KDL position-only IK ->
//   JointJog. JointSpace (X): sticks -> joint velocities (no IK); fallback for
//   unreachable poses.
enum class InputMode { JointSpace, Cartesian };

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

const std::string JOINT_TOPIC = "servo_node/delta_joint_cmds";
// Direct JTC topic - bypasses MoveIt's FollowJointTrajectory goal-and-wait.
const std::string GRIPPER_TRAJECTORY_TOPIC =
    "gripper_controller/joint_trajectory";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EE_FRAME_ID = "end_effector_link";
const double DEAD_MAN_SWITCH_THRESHOLD = -0.3;
const double JOY_DEADZONE = 0.05;
const double GRIPPER_MIN_POSE = -0.009;
const double GRIPPER_MAX_POSE = 0.015;
// Must exceed CM update period (10 ms) and one joy callback (~50 ms) for smooth
// motion.
const double GRIPPER_TRAJECTORY_TIME_FROM_START = 0.1; // seconds
const std::vector<std::string> JOINT_NAMES = {"joint1", "joint2", "joint3",
                                              "joint4"};

class Joy2Servo : public rclcpp::Node {
public:
  Joy2Servo();
  void InitializeMoveGroup();

private:
  void ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg);
  void PublishJointSpaceJog(const sensor_msgs::msg::Joy::SharedPtr msg);
  void PublishCartesianJog(const sensor_msgs::msg::Joy::SharedPtr msg);
  void PublishJointVelocities(const std::vector<double> &velocities);
  bool IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg);
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void JointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void MoveToDockPose();
  void MoveToHomePose();
  void UpdateInputMode(const sensor_msgs::msg::Joy::SharedPtr msg);
  void SetServoCommandTypeToJointJog();

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      gripper_traj_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr
      switch_cmd_type_srv_;
  moveit::planning_interface::MoveGroupInterfacePtr gripper_group_;
  moveit::planning_interface::MoveGroupInterfacePtr manipulator_group_;

  InputMode mode_ = InputMode::Cartesian;
  double cartesian_linear_velocity_;
  // Per-tick step duration; should be >= servo publish_period (~30 ms) and
  // ~joy autorepeat (50 ms).
  double cartesian_step_dt_;
  // Hard cap on |joint velocity| (rad/s) in Cartesian mode; without it,
  // position_only_ik branch jumps overshoot self_collision_proximity_threshold.
  double cartesian_max_joint_velocity_;
  // Seeds RobotState for Cartesian IK; bypasses MGI's lazy CurrentStateMonitor
  // warm-up.
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joy_mutex_;
};

} // namespace rosbot_moveit
