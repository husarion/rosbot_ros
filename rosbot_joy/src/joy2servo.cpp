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
#include <map>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbot_joy/servo/joy2servo.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <unordered_map>

namespace rosbot_joy::servo {

Joy2Servo::Joy2Servo() : Node("joy2servo") {
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC, ROS_QUEUE_SIZE);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Servo::JoyCb, this, std::placeholders::_1));

  switch_cmd_type_srv_ =
      this->create_client<moveit_msgs::srv::ServoCommandType>(
          "servo_node/switch_command_type");
}

void Joy2Servo::InitializeMoveGroup() {
  gripper_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "gripper");
  gripper_group_->setMaxVelocityScalingFactor(0.4);
  gripper_group_->setMaxAccelerationScalingFactor(0.2);
  manipulator_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "manipulator");
  manipulator_group_->setMaxVelocityScalingFactor(0.4);
  manipulator_group_->setMaxAccelerationScalingFactor(0.2);
}

void Joy2Servo::MoveToDockPose() {
  gripper_group_->setNamedTarget("Close");
  gripper_group_->move();
  gripper_group_->move(); // To make sure the action is finished

  manipulator_group_->setNamedTarget("Dock");
  manipulator_group_->move();
  manipulator_group_->move(); // To make sure the action is finished
}

void Joy2Servo::MoveToHomePose() {
  manipulator_group_->setNamedTarget("Home");
  manipulator_group_->move();
  manipulator_group_->move(); // To make sure the action is finished

  gripper_group_->setNamedTarget("Open");
  gripper_group_->move();
  gripper_group_->move(); // To make sure the action is finished
}

void Joy2Servo::ChangeCommandType(CommandType cmd_type) {
  auto request_ =
      std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request_->command_type = cmd_type;

  if (switch_cmd_type_srv_->wait_for_service(std::chrono::seconds(1))) {
    auto future = switch_cmd_type_srv_->async_send_request(
        request_, std::bind(&Joy2Servo::ChangeCommandTypeCallback, this,
                            std::placeholders::_1));
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Service " << switch_cmd_type_srv_->get_service_name()
                                  << " not available after waiting");
  }
}

void Joy2Servo::ChangeCommandTypeCallback(
    const rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture
        future) {
  static const std::unordered_map<CommandType, std::string> cmd_type_map = {
      {CommandType::NONE, "Uninitialized"},
      {CommandType::JOINT_JOG, "JointJog"},
      {CommandType::TWIST, "Twist"},
      {CommandType::POSE, "Pose"}};

  std::string req_cmd_type_str = cmd_type_map.find(req_cmd_type_)->second;

  if (future.get()->success) {
    cmd_type_ = req_cmd_type_;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Switched to input type: " << req_cmd_type_str);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Failed to switch input to: " << req_cmd_type_str);
  }
}

void Joy2Servo::ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg) {

  constexpr double AXIS_MIN = -1.0;
  constexpr double AXIS_MAX = 1.0;
  constexpr double POSITION_EPSILON = 0.001;

  double axis_value = msg->axes[Axis::LEFT_TRIGGER];
  double target_position =
      GRIPPER_MIN_POSE + ((axis_value - AXIS_MIN) / (AXIS_MAX - AXIS_MIN)) *
                             (GRIPPER_MAX_POSE - GRIPPER_MIN_POSE);

  if (std::abs(target_position - gripper_position_) > POSITION_EPSILON) {
    std::map<std::string, double> joint_positions;
    joint_positions["gripper_left_joint"] = target_position;
    gripper_group_->setJointValueTarget(joint_positions);
    gripper_group_->move();
    gripper_position_ = target_position;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Gripper moved to position: " << target_position);
  }
}

void Joy2Servo::ConvertAndPublishJoint(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

  joint_msg->joint_names = JOINT_NAMES;
  joint_msg->velocities = {
      msg->axes[Axis::LEFT_STICK_HORIZONTAL],
      msg->axes[Axis::LEFT_STICK_VERTICAL], // Invert axis to match joystick up
                                            // with joint up movement
      msg->axes[Axis::RIGHT_STICK_HORIZONTAL],
      -msg->axes[Axis::RIGHT_STICK_VERTICAL] // Invert axis to match joystick up
                                             // with joint up movement
  };

  joint_msg->header.stamp = this->now();
  joint_msg->header.frame_id = EE_FRAME_ID;
  joint_pub_->publish(std::move(joint_msg));
}

void Joy2Servo::ConvertAndPublishTwist(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->twist.linear.x = msg->axes[Axis::RIGHT_STICK_VERTICAL];
  twist_msg->twist.linear.z = msg->axes[Axis::LEFT_STICK_VERTICAL];
  twist_msg->twist.angular.y = msg->axes[Axis::LEFT_STICK_HORIZONTAL];

  twist_msg->header.stamp = this->now();
  twist_msg->header.frame_id = EE_FRAME_ID;
  twist_pub_->publish(std::move(twist_msg));
}

bool Joy2Servo::IsDeadManSwitch(const sensor_msgs::msg::Joy::SharedPtr msg) {
  return msg->axes[Axis::RIGHT_TRIGGER] <= DEAD_MAN_SWITCH_THRESHOLD;
}

void Joy2Servo::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(joy_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // Previous callback is still running, skip this one
    return;
  }

  UpdateReqCommand(msg);
  if (req_cmd_type_ != cmd_type_) {
    ChangeCommandType(req_cmd_type_);
  }

  if (IsDeadManSwitch(msg)) {
    if (msg->buttons[Button::BACK]) {
      MoveToDockPose();
    } else if (msg->buttons[Button::START]) {
      MoveToHomePose();
    } else if (msg->buttons[Button::RIGHT_BUMPER]) {
      ControlGripper(msg);
    } else if (req_cmd_type_ == CommandType::JOINT_JOG) {
      ConvertAndPublishJoint(msg);
    } else if (req_cmd_type_ == CommandType::TWIST) {
      ConvertAndPublishTwist(msg);
    }
  }
}

void Joy2Servo::UpdateReqCommand(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (msg->buttons[Button::X] ^ msg->buttons[Button::Y]) {
    // req_cmd_type_ = msg->buttons[Button::X] ? CommandType::JOINT_JOG :
    // CommandType::TWIST;
    // FIXME: singularity error
    req_cmd_type_ = CommandType::JOINT_JOG;
  }
}

} // namespace rosbot_joy::servo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosbot_joy::servo::Joy2Servo>();
  node->InitializeMoveGroup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
