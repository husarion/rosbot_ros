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

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class LEDStripPublisher : public rclcpp::Node {
public:
  LEDStripPublisher() : Node("led_strip_manager") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<sensor_msgs::msg::Image>("led_strip", qos);

    rainbow_ = CreateRainbowGradient();

    timer_ = create_wall_timer(
        40ms, std::bind(&LEDStripPublisher::TimerCallback, this));

    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "led_strip/enable",
        std::bind(&LEDStripPublisher::SetEnabled, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

private:
  using Rgb = std::array<std::uint8_t, 3>;

  // OpenCV stores the hue of an 8-bit HSV pixel in [0, 179] (half-degrees),
  // which the original Python implementation produced via cv2.cvtColor.
  // Replicated here with a plain fully-saturated HSV->RGB sweep to avoid an
  // OpenCV dependency.
  static Rgb HsvToRgb(double hue_opencv) {
    const double deg = hue_opencv * 2.0;
    const double sector = deg / 60.0;
    const double x = 1.0 - std::abs(std::fmod(sector, 2.0) - 1.0);

    double r = 0.0, g = 0.0, b = 0.0;
    switch (static_cast<int>(std::floor(sector)) % 6) {
    case 0:
      r = 1.0, g = x, b = 0.0;
      break;
    case 1:
      r = x, g = 1.0, b = 0.0;
      break;
    case 2:
      r = 0.0, g = 1.0, b = x;
      break;
    case 3:
      r = 0.0, g = x, b = 1.0;
      break;
    case 4:
      r = x, g = 0.0, b = 1.0;
      break;
    default:
      r = 1.0, g = 0.0, b = x;
      break;
    }

    return {static_cast<std::uint8_t>(r * 255.0),
            static_cast<std::uint8_t>(g * 255.0),
            static_cast<std::uint8_t>(b * 255.0)};
  }

  std::vector<Rgb> CreateRainbowGradient() const {
    std::vector<Rgb> gradient(gradient_resolution_);
    for (int i = 0; i < gradient_resolution_; ++i) {
      const double hue =
          std::floor(179.0 * (static_cast<double>(i) / gradient_resolution_));
      gradient[i] = HsvToRgb(hue);
    }
    return gradient;
  }

  std::vector<std::uint8_t> SampleGradient() const {
    std::vector<std::uint8_t> led_colors(num_leds_ * 3);
    for (int i = 0; i < num_leds_; ++i) {
      const double pos = std::fmod(
          position_ + i * gradient_resolution_ / static_cast<double>(num_leds_),
          gradient_resolution_);

      const int i0 = static_cast<int>(std::floor(pos));
      const int i1 = (i0 + 1) % gradient_resolution_;
      const double t = pos - i0;

      for (int c = 0; c < 3; ++c) {
        const double color = (1.0 - t) * rainbow_[i0][c] + t * rainbow_[i1][c];
        led_colors[i * 3 + c] = static_cast<std::uint8_t>(color);
      }
    }
    return led_colors;
  }

  void TimerCallback() {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.height = 1;
    msg.width = num_leds_;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = num_leds_ * 3;
    msg.data = SampleGradient();

    publisher_->publish(msg);

    position_ = std::fmod(position_ + speed_, gradient_resolution_);
  }

  // Cancelling the timer halts both gradient computation and publishing; the
  // animation resumes from its current position when re-enabled.
  void
  SetEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
      timer_->reset();
      response->message = "LED strip enabled";
    } else {
      timer_->cancel();
      response->message = "LED strip disabled";
    }
    response->success = true;
  }

  static constexpr int num_leds_ = 18;
  static constexpr int gradient_resolution_ = 180;
  static constexpr double speed_ = 1.5;

  double position_ = 0.0;
  std::vector<Rgb> rainbow_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDStripPublisher>());
  rclcpp::shutdown();
  return 0;
}
