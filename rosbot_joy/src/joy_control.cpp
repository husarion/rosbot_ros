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

#include <rosbot_joy/servo/joy_control.hpp>

namespace rosbot_joy::servo {

AxisControl::AxisControl(int axis_id, double axis_deadzone, double scaling,
                         bool inverted_control, double pressing_threshold) {
  axis_id_ = axis_id;
  axis_deadzone_ = axis_deadzone;
  inverted_ = inverted_control;
  scaling_ = inverted_ ? -scaling : scaling;
  pressing_threshold_ = pressing_threshold;
}

bool AxisControl::IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const {
  if (pressing_threshold_ != 0.0) {
    return (msg->axes[axis_id_] > pressing_threshold_) != inverted_;
  } else {
    return std::fabs(msg->axes[axis_id_]) > axis_deadzone_;
  }
}

double
AxisControl::GetControlValue(const sensor_msgs::msg::Joy::SharedPtr msg) const {
  return msg->axes[axis_id_] * scaling_;
}

DoubleButtonControl::DoubleButtonControl(int positive_button_id,
                                         int negative_button_id,
                                         double scaling) {
  positive_button_id_ = positive_button_id;
  negative_button_id_ = negative_button_id;
  scaling_ = scaling;
}

bool DoubleButtonControl::IsPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg) const {
  return msg->buttons[positive_button_id_] || msg->buttons[negative_button_id_];
}

double DoubleButtonControl::GetControlValue(
    const sensor_msgs::msg::Joy::SharedPtr msg) const {
  return (msg->buttons[positive_button_id_] -
          msg->buttons[negative_button_id_]) *
         scaling_;
}

SingleButtonControl::SingleButtonControl(int button_id, double scaling) {
  button_id_ = button_id;
  scaling_ = scaling;
}

bool SingleButtonControl::IsPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg) const {
  return msg->buttons[button_id_];
}

double SingleButtonControl::GetControlValue(
    const sensor_msgs::msg::Joy::SharedPtr msg) const {
  return msg->buttons[button_id_] * scaling_;
}

std::unique_ptr<JoyControl> JoyControlFactory(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        &param_itf,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &logging_itf,
    std::string param_namespace, double scaling) {
  param_itf->declare_parameter(param_namespace + ".control_type",
                               rclcpp::ParameterValue(""));
  std::string control_type =
      param_itf->get_parameter(param_namespace + ".control_type").as_string();

  std::unique_ptr<JoyControl> controller;

  if (control_type == "double_button") {
    param_itf->declare_parameter(param_namespace + ".positive_button_id",
                                 rclcpp::PARAMETER_INTEGER);
    param_itf->declare_parameter(param_namespace + ".negative_button_id",
                                 rclcpp::PARAMETER_INTEGER);
    int positive_button_id =
        param_itf->get_parameter(param_namespace + ".positive_button_id")
            .as_int();
    int negative_button_id =
        param_itf->get_parameter(param_namespace + ".negative_button_id")
            .as_int();
    controller = std::make_unique<DoubleButtonControl>(
        positive_button_id, negative_button_id, scaling);
  } else if (control_type == "single_button") {
    param_itf->declare_parameter(param_namespace + ".button_id",
                                 rclcpp::PARAMETER_INTEGER);
    int button_id =
        param_itf->get_parameter(param_namespace + ".button_id").as_int();
    controller = std::make_unique<SingleButtonControl>(button_id, scaling);
  } else if (control_type == "axis") {
    if (!param_itf->has_parameter("axis_deadzone")) {
      param_itf->declare_parameter("axis_deadzone",
                                   rclcpp::ParameterValue(0.05));
    }
    double axis_deadzone =
        param_itf->get_parameter("axis_deadzone").as_double();
    param_itf->declare_parameter(param_namespace + ".axis_id",
                                 rclcpp::PARAMETER_INTEGER);
    param_itf->declare_parameter(param_namespace + ".pressing_threshold",
                                 rclcpp::ParameterValue(0.0));
    param_itf->declare_parameter(param_namespace + ".inverted",
                                 rclcpp::ParameterValue(false));
    int axis_id =
        param_itf->get_parameter(param_namespace + ".axis_id").as_int();
    double pressing_threshold =
        param_itf->get_parameter(param_namespace + ".pressing_threshold")
            .as_double();
    bool inverted =
        param_itf->get_parameter(param_namespace + ".inverted").as_bool();
    controller = std::make_unique<AxisControl>(axis_id, axis_deadzone, scaling,
                                               inverted, pressing_threshold);
  } else {
    RCLCPP_ERROR_STREAM(logging_itf->get_logger(),
                        "Unknown control type "
                            << control_type << " in " << param_namespace
                            << ", currently supported types: single_button, "
                               "double_button, axis. "
                               "Please make sure that it is defined.");
    throw std::runtime_error("Unknown control type");
  }

  return controller;
}

} // namespace rosbot_joy::servo
