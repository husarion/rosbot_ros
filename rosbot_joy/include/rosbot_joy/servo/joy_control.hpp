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

#ifndef ROSBOT_JOY__SERVO__JOY_CONTROL_HPP_
#define ROSBOT_JOY__SERVO__JOY_CONTROL_HPP_

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

namespace rosbot_joy::servo {
class JoyControl {
public:
  virtual ~JoyControl() = default;

  virtual bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
  virtual double
  GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
};

class AxisControl : public JoyControl {
public:
  AxisControl(int axis_id, double axis_deadzone, double scaling = 1.0,
              bool inverted_control = false, double pressing_threshold = 0.0);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double
  GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int axis_id_;
  double axis_deadzone_;
  bool inverted_;
  double scaling_;
  double pressing_threshold_;
};

class DoubleButtonControl : public JoyControl {
public:
  DoubleButtonControl(int positive_button_id, int negative_button_id,
                      double scaling = 1.0);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double
  GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int positive_button_id_;
  int negative_button_id_;
  double scaling_;
};

class SingleButtonControl : public JoyControl {
public:
  SingleButtonControl(int button_id, double scaling = 1.0);

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override;
  double
  GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override;

private:
  int button_id_;
  double scaling_;
};

/**
 * @brief Reads ros2 parameters from given namespace and returns appropriate
 * JoyControl object
 *
 * @exception std::runtime_error when control_type class isn't supported (or it
 * wasn't defined)
 */
std::unique_ptr<JoyControl> JoyControlFactory(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        &param_itf,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &logging_itf,
    std::string param_namespace, double scaling = 1.0);

} // namespace rosbot_joy::servo

#endif // ROSBOT_JOY__SERVO__JOY_CONTROL_HPP_
