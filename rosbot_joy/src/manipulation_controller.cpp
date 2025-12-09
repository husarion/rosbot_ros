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

#include <rosbot_joy/servo/manipulation_controller.hpp>

#include <rclcpp/parameter.hpp>

namespace rosbot_joy::servo {

bool ManipulationController::CheckIfPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::map<std::string, std::unique_ptr<JoyControl>> &controls) {
  for (const auto &c : controls) {
    if (c.second->IsPressed(msg)) {
      return true;
    }
  }
  return false;
}

std::vector<double> ManipulationController::CalculateCommand(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::vector<std::string> &cmd_names,
    const std::map<std::string, std::unique_ptr<JoyControl>> &controls) {
  std::vector<double> cmds(cmd_names.size(), 0.0);
  for (auto const &c : controls) {
    if (c.second->IsPressed(msg)) {
      auto name_it = std::find(cmd_names.begin(), cmd_names.end(), c.first);
      int name_idx = std::distance(cmd_names.begin(), name_it);
      cmds[name_idx] = c.second->GetControlValue(msg);
    }
  }
  return cmds;
}

JointController::JointController(const rclcpp::Node::SharedPtr &node) {
  clock_itf_ = node->get_node_clock_interface();
  ParseParameters(node);

  joint_cmds_pub_ = node->create_publisher<control_msgs::msg::JointJog>(
      "servo_node/delta_joint_cmds", 10);
};

bool JointController::Process(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (CheckIfPressed(msg, manipulator_joint_controls_)) {
    std::vector<double> cmds =
        CalculateCommand(msg, joint_names_, manipulator_joint_controls_);
    SendJointCommand(cmds, clock_itf_->get_clock()->now());
    return true;
  }
  return false;
}

void JointController::Stop() {
  SendJointCommand(std::vector<double>(joint_names_.size(), 0.0),
                   clock_itf_->get_clock()->now());
}

void JointController::ParseParameters(const rclcpp::Node::SharedPtr &node) {
  node->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  try {
    joint_names_ = node->get_parameter("joint_names").as_string_array();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Required parameter not defined: " << e.what());
    throw e;
  }
  for (const auto &joint_name : joint_names_) {
    std::string param_namespace = "joints_control." + joint_name;

    node->declare_parameter(param_namespace + ".max_velocity",
                            rclcpp::PARAMETER_DOUBLE);
    try {
      double max_velocity =
          node->get_parameter(param_namespace + ".max_velocity").as_double();
      manipulator_joint_controls_[joint_name] = JoyControlFactory(
          node->get_node_parameters_interface(),
          node->get_node_logging_interface(), param_namespace, max_velocity);
    } catch (const rclcpp::exceptions::ParameterUninitializedException &e) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Required parameter not defined: " << e.what());
      throw e;
    }
  }
}

void JointController::SendJointCommand(
    const std::vector<double> &cmds,
    const builtin_interfaces::msg::Time &timestamp) {
  control_msgs::msg::JointJog joint_cmd_msg;
  joint_cmd_msg.header.stamp = timestamp;
  joint_cmd_msg.duration = 0.0;
  joint_cmd_msg.joint_names = joint_names_;
  joint_cmd_msg.velocities = cmds;
  joint_cmds_pub_->publish(joint_cmd_msg);
}

CartesianController::CartesianController(const rclcpp::Node::SharedPtr &node)
    : cartesian_cmd_names_{
          "linear_x",  "linear_y",  "linear_z",
          "angular_x", "angular_y", "angular_z",
      } {
  clock_itf_ = node->get_node_clock_interface();
  ParseParameters(node);

  twist_cmds_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_node/delta_twist_cmds", 10);
};

bool CartesianController::Process(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (CheckIfPressed(msg, manipulator_cartesian_controls_)) {
    std::vector<double> cmds = CalculateCommand(
        msg, cartesian_cmd_names_, manipulator_cartesian_controls_);
    SendCartesianCommand(cmds, clock_itf_->get_clock()->now());
    return true;
  }
  return false;
}

void CartesianController::Stop() {
  SendCartesianCommand(std::vector<double>(cartesian_cmd_names_.size(), 0.0),
                       clock_itf_->get_clock()->now());
}

void CartesianController::ParseParameters(const rclcpp::Node::SharedPtr &node) {
  node->declare_parameter("cartesian_control_reference_frame",
                          rclcpp::PARAMETER_STRING);
  node->declare_parameter("cartesian_control_names",
                          rclcpp::PARAMETER_STRING_ARRAY);
  try {
    cartesian_control_reference_frame_ =
        node->get_parameter("cartesian_control_reference_frame").as_string();
    cartesian_control_names_ =
        node->get_parameter("cartesian_control_names").as_string_array();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Required parameter not defined: " << e.what());
    throw e;
  }
  for (const auto &cartesian_control_name : cartesian_control_names_) {
    if (std::find(cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(),
                  cartesian_control_name) == cartesian_cmd_names_.end()) {
      RCLCPP_ERROR(
          node->get_logger(),
          "Unknown cartesian control type {cartesian_control_name},"
          " currently supported names: {self.cartesian_control_names}");
      continue;
    }
    std::string param_namespace = "cartesian_control." + cartesian_control_name;

    node->declare_parameter<double>(param_namespace + ".max_velocity",
                                    rclcpp::PARAMETER_DOUBLE);
    try {
      double max_velocity =
          node->get_parameter(param_namespace + ".max_velocity").as_double();
      manipulator_cartesian_controls_[cartesian_control_name] =
          JoyControlFactory(node->get_node_parameters_interface(),
                            node->get_node_logging_interface(), param_namespace,
                            max_velocity);
    } catch (const rclcpp::exceptions::ParameterUninitializedException &e) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Required parameter not defined: " << e.what());
      throw e;
    }
  }
}

void CartesianController::SendCartesianCommand(
    const std::vector<double> &cmds,
    const builtin_interfaces::msg::Time &timestamp) {
  geometry_msgs::msg::TwistStamped cartesian_cmd_msg;
  cartesian_cmd_msg.header.stamp = timestamp;
  cartesian_cmd_msg.header.frame_id = cartesian_control_reference_frame_;
  cartesian_cmd_msg.twist.linear.x = cmds[0];
  cartesian_cmd_msg.twist.linear.y = cmds[1];
  cartesian_cmd_msg.twist.linear.z = cmds[2];
  cartesian_cmd_msg.twist.angular.x = cmds[3];
  cartesian_cmd_msg.twist.angular.y = cmds[4];
  cartesian_cmd_msg.twist.angular.z = cmds[5];

  twist_cmds_pub_->publish(cartesian_cmd_msg);
}

ManipulatorMoveGroupController::ManipulatorMoveGroupController(
    const rclcpp::Node::SharedPtr &node) {

  auto manipulator_options =
      MGI::Options("manipulator", "robot_description", node->get_namespace());
  manipulator_group_ = std::make_unique<MGI>(node, manipulator_options);

  auto gripper_options =
      MGI::Options("gripper", "robot_description", node->get_namespace());
  gripper_group_ = std::make_unique<MGI>(node, gripper_options);

  ParseParameters(node);

  double velocity_scaling_factor =
      node->get_parameter("velocity_scaling_factor").as_double();
  double acceleration_scaling_factor =
      node->get_parameter("acceleration_scaling_factor").as_double();
  manipulator_group_->setMaxVelocityScalingFactor(velocity_scaling_factor);
  manipulator_group_->setMaxAccelerationScalingFactor(
      acceleration_scaling_factor);
}

bool ManipulatorMoveGroupController::Process(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (dock_manipulator_->IsPressed(msg)) {
    if (!is_action_executing_) {
      is_action_executing_ = true;
      MoveToDockPose();
    }
    return true;
  } else if (home_manipulator_->IsPressed(msg)) {
    if (!is_action_executing_) {
      is_action_executing_ = true;
      MoveToHomePose();
    }
    return true;
  } else if (gripper_trigger_->IsPressed(msg)) {
    is_action_executing_ = true;
    ControlGripper(msg);
    return true;
  }
  is_action_executing_ = false;
  return false;
}

void ManipulatorMoveGroupController::ParseParameters(
    const rclcpp::Node::SharedPtr &node) {
  dock_manipulator_ =
      JoyControlFactory(node->get_node_parameters_interface(),
                        node->get_node_logging_interface(), "dock_manipulator");
  home_manipulator_ =
      JoyControlFactory(node->get_node_parameters_interface(),
                        node->get_node_logging_interface(), "home_manipulator");
  gripper_cmd_ = JoyControlFactory(node->get_node_parameters_interface(),
                                   node->get_node_logging_interface(),
                                   "gripper_control.control");
  gripper_trigger_ = JoyControlFactory(node->get_node_parameters_interface(),
                                       node->get_node_logging_interface(),
                                       "gripper_control.trigger");
  node->declare_parameter<double>("velocity_scaling_factor", 0.1);
  node->declare_parameter<double>("acceleration_scaling_factor", 0.1);
  node->declare_parameter<double>("gripper_control.min_position", -0.009);
  node->declare_parameter<double>("gripper_control.max_position", 0.019);
  node->declare_parameter<std::string>("joint_name", "gripper_left_joint");
  try {
    gripper_min_pose_ =
        node->get_parameter("gripper_control.min_position").as_double();
    gripper_max_pose_ =
        node->get_parameter("gripper_control.max_position").as_double();
    joint_name_gripper_ = node->get_parameter("joint_name").as_string();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Required parameter not defined: " << e.what());
    throw e;
  }
}

void ManipulatorMoveGroupController::ControlGripper(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  constexpr double AXIS_MIN = -1.0;
  constexpr double AXIS_MAX = 1.0;

  double axis_value = gripper_trigger_->GetControlValue(msg);
  double target_position =
      gripper_min_pose_ + ((axis_value - AXIS_MIN) / (AXIS_MAX - AXIS_MIN)) *
                              (gripper_max_pose_ - gripper_min_pose_);

  std::map<std::string, double> joint_positions;
  joint_positions["gripper_left_joint"] = target_position;
  gripper_group_->setJointValueTarget(joint_positions);
  gripper_group_->move();
  gripper_position_ = target_position;
}

void ManipulatorMoveGroupController::MoveToDockPose() {
  gripper_group_->setNamedTarget("Close");
  gripper_group_->move();
  gripper_group_->move(); // To make sure the action is finished
  manipulator_group_->setNamedTarget("Dock");
  manipulator_group_->move();
  manipulator_group_->move(); // To make sure the action is finished
}

void ManipulatorMoveGroupController::MoveToHomePose() {
  manipulator_group_->setNamedTarget("Home");
  manipulator_group_->move();
  manipulator_group_->move(); // To make sure the action is finished
  gripper_group_->setNamedTarget("Open");
  gripper_group_->move();
  gripper_group_->move(); // To make sure the action is finished
}

} // namespace rosbot_joy::servo
