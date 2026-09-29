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

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr char kJoint[] = "base-finger-joint";
constexpr double kLower = -0.010;
constexpr double kUpper = 0.019;

class ClampStartStateBoundsTest : public ::testing::Test {
protected:
  void SetUp() override {
    moveit::core::RobotModelBuilder builder("gripper_robot", "base");
    builder.addChain("base->finger", "prismatic");
    builder.addGroup({"finger"}, {}, "gripper");
    ASSERT_TRUE(builder.isValid());
    model_ = builder.build();

    moveit::core::VariableBounds bounds;
    bounds.position_bounded_ = true;
    bounds.min_position_ = kLower;
    bounds.max_position_ = kUpper;
    model_->getJointModel(kJoint)->setVariableBounds(kJoint, bounds);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

    node_ = std::make_shared<rclcpp::Node>("test_clamp_start_state_bounds");
    adapter_ =
        loader_.createSharedInstance("rosbot_moveit/ClampStartStateBounds");
    adapter_->initialize(node_, "ompl");
  }

  double Adapt(double start) {
    planning_interface::MotionPlanRequest req;
    req.group_name = "gripper";
    req.start_state.joint_state.name = {kJoint};
    req.start_state.joint_state.position = {start};
    EXPECT_EQ(adapter_->adapt(scene_, req).val,
              moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    const auto &names = req.start_state.joint_state.name;
    const auto it = std::find(names.begin(), names.end(), kJoint);
    return req.start_state.joint_state.position[it - names.begin()];
  }

  pluginlib::ClassLoader<planning_interface::PlanningRequestAdapter> loader_{
      "moveit_core", "planning_interface::PlanningRequestAdapter"};
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<planning_interface::PlanningRequestAdapter> adapter_;
};

TEST_F(ClampStartStateBoundsTest, ClampsWithinToleranceOntoTheLimit) {
  // HW: gripper resting past its lower limit after Close + torque off.
  EXPECT_DOUBLE_EQ(Adapt(-0.0107), kLower);
  EXPECT_DOUBLE_EQ(Adapt(0.0191), kUpper);
}

TEST_F(ClampStartStateBoundsTest,
       LeavesLargeViolationsForCheckStartStateBounds) {
  EXPECT_DOUBLE_EQ(Adapt(-0.015), -0.015);
  EXPECT_DOUBLE_EQ(Adapt(0.025), 0.025);
}

TEST_F(ClampStartStateBoundsTest, LeavesInBoundsStatesUntouched) {
  EXPECT_DOUBLE_EQ(Adapt(0.005), 0.005);
  EXPECT_DOUBLE_EQ(Adapt(kLower), kLower);
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
