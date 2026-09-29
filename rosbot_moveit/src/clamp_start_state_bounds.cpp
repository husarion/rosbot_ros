// Copyright 2026 Husarion sp. z o.o.
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

// Pulls a start state that sits marginally outside the joint limits back onto
// the limit, so CheckStartStateBounds (which has no tolerance for revolute or
// prismatic joints in MoveIt 2.12.4) does not reject the request. A gripper at
// rest past its URDF limit (mechanical stops lie beyond it, and differ per
// unit) otherwise aborts every plan. Goals stay bounded by the URDF limits;
// only the start state is corrected, and only within the tolerance.

#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rosbot_moveit {

class ClampStartStateBounds
    : public planning_interface::PlanningRequestAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr &node,
                  const std::string &parameter_namespace) override {
    const std::string param =
        parameter_namespace + ".start_state_max_bounds_error";
    if (!node->has_parameter(param)) {
      node->declare_parameter(param, kDefaultTolerance);
    }
    tolerance_ = node->get_parameter(param).as_double();
    logger_ = node->get_logger().get_child("clamp_start_state_bounds");
  }

  [[nodiscard]] std::string getDescription() const override {
    return "ClampStartStateBounds";
  }

  [[nodiscard]] moveit::core::MoveItErrorCode
  adapt(const planning_scene::PlanningSceneConstPtr &planning_scene,
        planning_interface::MotionPlanRequest &req) const override {
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(),
                                            req.start_state, start_state);

    const auto &robot_model = planning_scene->getRobotModel();
    const std::vector<const moveit::core::JointModel *> &joint_models =
        robot_model->hasJointModelGroup(req.group_name)
            ? robot_model->getJointModelGroup(req.group_name)->getJointModels()
            : robot_model->getJointModels();

    bool clamped = false;
    for (const moveit::core::JointModel *joint_model : joint_models) {
      const auto type = joint_model->getType();
      if (type != moveit::core::JointModel::REVOLUTE &&
          type != moveit::core::JointModel::PRISMATIC) {
        continue;
      }
      const moveit::core::VariableBounds &bounds =
          joint_model->getVariableBounds()[0];
      if (!bounds.position_bounded_) {
        continue;
      }

      const double position = start_state.getJointPositions(joint_model)[0];
      double target = position;
      if (position < bounds.min_position_ &&
          bounds.min_position_ - position <= tolerance_) {
        target = bounds.min_position_;
      } else if (position > bounds.max_position_ &&
                 position - bounds.max_position_ <= tolerance_) {
        target = bounds.max_position_;
      }
      if (target == position) {
        continue;
      }

      start_state.setJointPositions(joint_model, &target);
      clamped = true;
      RCLCPP_WARN(logger_,
                  "Start state of '%s' is %.6f, outside its limits by less "
                  "than %.4f; planning from %.6f",
                  joint_model->getName().c_str(), position, tolerance_, target);
    }

    if (clamped) {
      start_state.update();
      moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
    }

    moveit::core::MoveItErrorCode status(
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    status.source = getDescription();
    return status;
  }

private:
  // Metres or radians, like the joint it applies to. Well above the few-tick
  // excursions seen on HW, far below anything that would hide a real fault.
  static constexpr double kDefaultTolerance = 0.002;

  double tolerance_{kDefaultTolerance};
  rclcpp::Logger logger_{rclcpp::get_logger("clamp_start_state_bounds")};
};

} // namespace rosbot_moveit

PLUGINLIB_EXPORT_CLASS(rosbot_moveit::ClampStartStateBounds,
                       planning_interface::PlanningRequestAdapter)
