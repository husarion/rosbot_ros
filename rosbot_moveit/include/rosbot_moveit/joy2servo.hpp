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

namespace rosbot_moveit {

// Internal mode toggle in joy2servo. *Both* modes publish JointJog to servo:
//   * Cartesian  (default, button Y) - sticks integrate EE-frame velocity
//                  into a target pose; joy2servo runs KDL position-only IK
//                  on it and publishes the resulting joint deltas as
//                  JointJog. We do the IK ourselves because moveit_servo's
//                  POSE path runs an unconditional singularity guard on the
//                  full 6xN Jacobian (rank-deficient by construction on
//                  <6 DoF arms, see moveit_msgs#185), which halts every
//                  motion on a 4-DoF arm even with position_only_ik=true.
//                  The JointJog path does not run that guard.
//   * JointSpace (button X)         - sticks map 1:1 to joint velocities
//                  (no IK), useful as a low-level fallback / for joints
//                  unreachable in Cartesian.
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
const size_t ROS_QUEUE_SIZE = 10;
const std::string EE_FRAME_ID = "end_effector_link";
const double DEAD_MAN_SWITCH_THRESHOLD = -0.3;
const double JOY_DEADZONE = 0.05;
const double GRIPPER_MIN_POSE = -0.009;
const double GRIPPER_MAX_POSE = 0.015;
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
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr
      switch_cmd_type_srv_;
  moveit::planning_interface::MoveGroupInterfacePtr gripper_group_;
  moveit::planning_interface::MoveGroupInterfacePtr manipulator_group_;

  InputMode mode_ = InputMode::Cartesian;
  double gripper_position_;
  double cartesian_linear_velocity_;
  // Per-tick step duration used as both the EE-frame offset multiplier and
  // the velocity divisor in Cartesian mode. Should be ≥ servo's
  // publish_period (~30 ms) and ≈ joy's autorepeat period (50 ms by default
  // in rosbot_joy/config.yaml).
  double cartesian_step_dt_;
  // Latest /joint_states snapshot used to seed RobotState in Cartesian mode -
  // avoids MGI's lazy CurrentStateMonitor warm-up which logs "Didn't receive
  // robot state ... within 1.0 seconds" on the first IK call.
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joy_mutex_;
};

} // namespace rosbot_moveit
