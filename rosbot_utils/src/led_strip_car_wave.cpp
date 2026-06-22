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

#include <algorithm>
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

class LEDStripWavePublisher : public rclcpp::Node {
public:
  LEDStripWavePublisher() : Node("led_strip_manager") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    pub_ = create_publisher<sensor_msgs::msg::Image>("led_strip", qos);
    timer_ =
        create_wall_timer(40ms, std::bind(&LEDStripWavePublisher::Tick, this));

    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "led_strip/enable",
        std::bind(&LEDStripWavePublisher::SetEnabled, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  // Python's % keeps the sign of the divisor; phase starts negative, so a plain
  // std::fmod would yield a negative result and shift the wave.
  static double PythonMod(double value, double modulus) {
    const double result = std::fmod(value, modulus);
    return result < 0.0 ? result + modulus : result;
  }

  void Tick() {
    phase_ = PythonMod(phase_ + kSpeed, kMaxPhase);

    std::vector<std::uint8_t> pixels(kNumLeds * 3);
    for (int i = 0; i < kNumLeds; ++i) {
      double dist;
      if (i < 5) {
        dist = std::abs(i - 0.5);
      } else if (i > 12) {
        dist = std::abs(i - 16.5);
      } else {
        dist = std::abs(i - 8.5);
      }

      const bool white = (i >= 5 && i <= 12);
      const std::array<int, 3> color = white ? std::array<int, 3>{255, 255, 255}
                                             : std::array<int, 3>{255, 0, 0};

      const double offset = phase_ - kWaveWidth;
      double intensity =
          std::clamp(1.0 - std::abs(dist - offset) / kWaveWidth, 0.0, 1.0);
      if (phase_ < kWaveWidth) {
        intensity *= phase_ / kWaveWidth;
      }

      for (int c = 0; c < 3; ++c) {
        pixels[i * 3 + c] = static_cast<std::uint8_t>(color[c] * intensity);
      }
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.height = 1;
    msg.width = kNumLeds;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = kNumLeds * 3;
    msg.data = std::move(pixels);

    pub_->publish(msg);
  }

  // Cancelling the timer halts both wave computation and publishing; the
  // animation resumes from its current phase when re-enabled.
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

  static constexpr int kNumLeds = 18;
  static constexpr double kSpeed = 0.14;
  static constexpr double kWaveWidth = 2.3;
  static constexpr int kMaxPhase = 9; // Depend on number of leds in segment

  double phase_ = -5.0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDStripWavePublisher>());
  rclcpp::shutdown();
  return 0;
}
