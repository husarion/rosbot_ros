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
#include <cmath>
#include <memory>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbot_moveit/arm_pose_mover.hpp>
#include <rosbot_moveit/joy2servo.hpp>
#include <string>

namespace rosbot_moveit {

Joy2Servo::Joy2Servo() : Node("joy2servo") {
  cartesian_linear_velocity_ =
      this->declare_parameter<double>("cartesian_linear_velocity", 0.1);
  cartesian_step_dt_ =
      this->declare_parameter<double>("cartesian_step_dt", 0.05);
  cartesian_max_joint_velocity_ =
      this->declare_parameter<double>("cartesian_max_joint_velocity", 1.0);

  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC, ROS_QUEUE_SIZE);

  gripper_traj_pub_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          GRIPPER_TRAJECTORY_TOPIC, ROS_QUEUE_SIZE);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Servo::JoyCb, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&Joy2Servo::JointStateCb, this, std::placeholders::_1));

  switch_cmd_type_srv_ =
      this->create_client<moveit_msgs::srv::ServoCommandType>(
          "servo_node/switch_command_type");
}

void Joy2Servo::JointStateCb(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = msg;
}

void Joy2Servo::SetServoCommandTypeToJointJog() {
  // moveit_servo defaults expected_command_type_ to "unset" in jazzy 2.12.4
  // and rejects messages until switch_command_type is called.
  if (!switch_cmd_type_srv_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       switch_cmd_type_srv_->get_service_name()
                           << " not available after 5s; servo will reject "
                              "JointJog messages until it appears");
    return;
  }
  auto request =
      std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type =
      moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
  switch_cmd_type_srv_->async_send_request(
      request,
      [logger = this->get_logger()](
          rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture
              future) {
        if (!future.get()->success) {
          RCLCPP_WARN(logger, "servo refused switch_command_type=JOINT_JOG");
        }
      });
}

void Joy2Servo::InitializeMoveGroup() {
  // MGI builds FQN topics via rclcpp::names::append(move_group_namespace, ...),
  // bypassing the node's namespace - pass it explicitly.
  const std::string move_group_namespace = this->get_namespace();

  moveit::planning_interface::MoveGroupInterface::Options gripper_options(
      "gripper",
      moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
      move_group_namespace);
  gripper_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), gripper_options);
  gripper_group_->setWorkspace(
      -ARM_WORKSPACE_HALF_EDGE, -ARM_WORKSPACE_HALF_EDGE,
      -ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE,
      ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE);
  gripper_group_->setMaxVelocityScalingFactor(0.4);
  gripper_group_->setMaxAccelerationScalingFactor(0.2);

  moveit::planning_interface::MoveGroupInterface::Options manipulator_options(
      "manipulator",
      moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
      move_group_namespace);
  manipulator_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), manipulator_options);
  manipulator_group_->setWorkspace(
      -ARM_WORKSPACE_HALF_EDGE, -ARM_WORKSPACE_HALF_EDGE,
      -ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE,
      ARM_WORKSPACE_HALF_EDGE, ARM_WORKSPACE_HALF_EDGE);
  manipulator_group_->setMaxVelocityScalingFactor(0.4);
  manipulator_group_->setMaxAccelerationScalingFactor(0.2);

  SetServoCommandTypeToJointJog();
}

void Joy2Servo::MoveToDockPose() {
  gripper_group_->setNamedTarget("Close");
  gripper_group_->move();

  manipulator_group_->setNamedTarget("Dock");
  manipulator_group_->move();
  // Band-aid: MGI->move() returns success before arm reaches target on first
  // call (CurrentStateMonitor staleness). See FOLLOWUP_PLAN.md C4.
  manipulator_group_->move();
}

void Joy2Servo::MoveToHomePose() {
  manipulator_group_->setNamedTarget("Home");
  manipulator_group_->move();
  manipulator_group_->move(); // See note in MoveToDockPose.

  gripper_group_->setNamedTarget("Open");
  gripper_group_->move();
}

void Joy2Servo::ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg) {
  constexpr double AXIS_MIN = -1.0;
  constexpr double AXIS_MAX = 1.0;

  const double axis_value = msg->axes[Axis::LEFT_TRIGGER];
  const double target_position =
      GRIPPER_MIN_POSE + ((axis_value - AXIS_MIN) / (AXIS_MAX - AXIS_MIN)) *
                             (GRIPPER_MAX_POSE - GRIPPER_MIN_POSE);

  // Single-point JointTrajectory: each new traj replaces the previous one,
  // avoiding the goal cancel/restart stair-step of GripperActionController.
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->now();
  traj.joint_names = {"gripper_left_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_position};
  point.time_from_start =
      rclcpp::Duration::from_seconds(GRIPPER_TRAJECTORY_TIME_FROM_START);
  traj.points.push_back(point);
  gripper_traj_pub_->publish(traj);
}

void Joy2Servo::PublishJointVelocities(const std::vector<double> &velocities) {
  auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
  joint_msg->joint_names = JOINT_NAMES;
  joint_msg->velocities = velocities;
  joint_msg->header.stamp = this->now();
  joint_msg->header.frame_id = EE_FRAME_ID;
  joint_pub_->publish(std::move(joint_msg));
}

void Joy2Servo::PublishJointSpaceJog(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  PublishJointVelocities({
      msg->axes[Axis::LEFT_STICK_HORIZONTAL],
      msg->axes[Axis::LEFT_STICK_VERTICAL],
      msg->axes[Axis::RIGHT_STICK_HORIZONTAL],
      -msg->axes[Axis::RIGHT_STICK_VERTICAL], // stick up = joint up
  });
}

void Joy2Servo::PublishCartesianJog(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Param-driven dt (not joy callback period) so vel scales with stick only.
  const double cartesian_step_dt = cartesian_step_dt_;

  auto deadzone = [](double v) { return std::abs(v) < JOY_DEADZONE ? 0.0 : v; };
  const double sx = deadzone(msg->axes[Axis::RIGHT_STICK_VERTICAL]);
  const double sy = deadzone(msg->axes[Axis::RIGHT_STICK_HORIZONTAL]);
  const double sz = deadzone(msg->axes[Axis::LEFT_STICK_VERTICAL]);

  if (sx == 0.0 && sy == 0.0 && sz == 0.0) {
    PublishJointVelocities(std::vector<double>(JOINT_NAMES.size(), 0.0));
    return;
  }

  sensor_msgs::msg::JointState::SharedPtr joint_state;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state = latest_joint_state_;
  }
  if (!joint_state) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Cartesian mode: no /joint_states yet");
    return;
  }

  // Seed from latest /joint_states - bypasses MGI's lazy CurrentStateMonitor.
  moveit::core::RobotState state(manipulator_group_->getRobotModel());
  state.setToDefaultValues();
  state.setVariablePositions(joint_state->name, joint_state->position);
  state.update();
  const moveit::core::JointModelGroup *jmg =
      state.getJointModelGroup("manipulator");

  const Eigen::Isometry3d ee_in_world =
      state.getGlobalLinkTransform(EE_FRAME_ID);
  const Eigen::Vector3d delta_in_ee(
      sx * cartesian_linear_velocity_ * cartesian_step_dt,
      sy * cartesian_linear_velocity_ * cartesian_step_dt,
      sz * cartesian_linear_velocity_ * cartesian_step_dt);
  Eigen::Isometry3d target = ee_in_world;
  target.translation() += ee_in_world.linear() * delta_in_ee;

  std::vector<double> current_joints;
  state.copyJointGroupPositions(jmg, current_joints);

  geometry_msgs::msg::Pose target_msg;
  target_msg.position.x = target.translation().x();
  target_msg.position.y = target.translation().y();
  target_msg.position.z = target.translation().z();
  const Eigen::Quaterniond q(target.linear());
  target_msg.orientation.x = q.x();
  target_msg.orientation.y = q.y();
  target_msg.orientation.z = q.z();
  target_msg.orientation.w = q.w();
  // position_only_ik (kinematics.yaml) zero-weights orientation - needed on
  // 4-DoF arm.
  if (!state.setFromIK(jmg, target_msg, EE_FRAME_ID, 0.05)) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Cartesian mode: IK failed for target ("
                                    << target_msg.position.x << ", "
                                    << target_msg.position.y << ", "
                                    << target_msg.position.z << ")");
    // Halt immediately; otherwise servo coasts ~280 ms
    // (incoming_command_timeout
    // + num_outgoing_halt_msgs_to_publish) at the last published velocity.
    PublishJointVelocities(std::vector<double>(JOINT_NAMES.size(), 0.0));
    return;
  }

  std::vector<double> target_joints;
  state.copyJointGroupPositions(jmg, target_joints);

  // JointJog path skips servo's singularity guard (unlike POSE/TWIST).
  std::vector<double> velocities(current_joints.size());
  for (size_t i = 0; i < current_joints.size(); ++i) {
    velocities[i] = (target_joints[i] - current_joints[i]) / cartesian_step_dt;
  }

  // Uniform clamp: scale whole vector to preserve Cartesian direction.
  double max_abs = 0.0;
  for (double v : velocities) {
    max_abs = std::max(max_abs, std::abs(v));
  }
  if (max_abs > cartesian_max_joint_velocity_) {
    const double scale = cartesian_max_joint_velocity_ / max_abs;
    for (double &v : velocities) {
      v *= scale;
    }
  }

  PublishJointVelocities(velocities);
}

bool Joy2Servo::IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg) {
  return msg->axes[Axis::RIGHT_TRIGGER] <= DEAD_MAN_SWITCH_THRESHOLD;
}

void Joy2Servo::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(joy_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return; // previous callback still running
  }

  UpdateInputMode(msg);

  if (IsDeadManSwitch(msg)) {
    if (msg->buttons[Button::BACK]) {
      MoveToDockPose();
    } else if (msg->buttons[Button::START]) {
      MoveToHomePose();
    } else if (msg->buttons[Button::RIGHT_BUMPER]) {
      ControlGripper(msg);
    } else if (mode_ == InputMode::JointSpace) {
      PublishJointSpaceJog(msg);
    } else if (mode_ == InputMode::Cartesian) {
      PublishCartesianJog(msg);
    }
  }
}

void Joy2Servo::UpdateInputMode(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // X => JointSpace, Y => Cartesian; XOR so pressing both is a no-op.
  if (!(msg->buttons[Button::X] ^ msg->buttons[Button::Y])) {
    return;
  }
  const InputMode requested =
      msg->buttons[Button::X] ? InputMode::JointSpace : InputMode::Cartesian;
  if (requested == mode_) {
    return;
  }
  mode_ = requested;
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Switched to input mode: "
          << (mode_ == InputMode::JointSpace ? "JointSpace" : "Cartesian"));
}

} // namespace rosbot_moveit

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosbot_moveit::Joy2Servo>();
  node->InitializeMoveGroup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
