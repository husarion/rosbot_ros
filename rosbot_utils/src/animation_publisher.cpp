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

// Parameter-selectable LED animation publisher for the ROSbot strip.
//
// Each animation is a PNG (row = frame, column = LED) plus an optional sidecar
// YAML (<name>.yaml) carrying `frequency` (Hz), `brightness` (0..1) and an
// optional `color` (#RRGGBB) tint. Animations are auto-discovered by globbing a
// shipped dir (share/rosbot_utils/animations) and an optional user dir; the
// filename stem is the animation name. The active animation is the
// `current_animation` node parameter — validated on set against the loaded set
// (or the reserved `none`, which publishes nothing so the firmware idle
// animation shows). Frames are published as sensor_msgs/Image (1 x led_count,
// rgb8, BEST_EFFORT) at the animation's frequency; a PNG row wider than
// led_count is cropped to the first led_count columns, narrower rows are padded
// black. Supersedes the retired led_strip_rainbow / led_strip_car_wave nodes.

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {
constexpr char kReservedNone[] = "none";
constexpr double kDefaultFrequency = 25.0;
constexpr double kDefaultBrightness = 1.0;
} // namespace

class AnimationPublisher : public rclcpp::Node {
public:
  AnimationPublisher() : Node("animation_publisher") {
    led_count_ = static_cast<int>(declare_parameter<int>("led_count", 18));
    if (led_count_ <= 0 || led_count_ > 18) {
      RCLCPP_WARN(get_logger(),
                  "led_count %d out of range (1..18); clamping to 18",
                  led_count_);
      led_count_ = std::clamp(led_count_, 1, 18);
    }
    const auto user_dir = declare_parameter<std::string>("user_animations_dir", "");

    LoadAnimations(user_dir);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<sensor_msgs::msg::Image>("led_strip", qos);

    // Validate `current_animation` on every set; apply the switch in the same
    // callback once the value is known-good (the reserved `none` and any loaded
    // animation are accepted, anything else is rejected with a reason).
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &p : params) {
            if (p.get_name() != "current_animation") {
              continue;
            }
            const std::string name = p.as_string();
            if (name != kReservedNone && animations_.find(name) == animations_.end()) {
              result.successful = false;
              result.reason = "unknown animation '" + name + "'; available: " +
                              AvailableNames();
              return result;
            }
            Activate(name);
          }
          return result;
        });

    const auto initial =
        declare_parameter<std::string>("current_animation", "car_wave");
    if (initial != kReservedNone && animations_.find(initial) == animations_.end()) {
      RCLCPP_WARN(get_logger(),
                  "current_animation '%s' not found; falling back to '%s'. "
                  "Available: %s",
                  initial.c_str(), kReservedNone, AvailableNames().c_str());
      Activate(kReservedNone);
    } else {
      Activate(initial);
    }
  }

private:
  // One animation: a sequence of ready-to-publish rgb8 frames (led_count*3
  // bytes each) plus the publish/frame-advance rate.
  struct Animation {
    std::vector<std::vector<std::uint8_t>> frames;
    double frequency = kDefaultFrequency;
  };

  struct Sidecar {
    double frequency = kDefaultFrequency;
    double brightness = kDefaultBrightness;
    bool has_color = false;
    std::uint8_t color[3] = {0, 0, 0};
  };

  // Minimal flat `key: value` reader for the sidecar (our own controlled
  // schema — not arbitrary YAML): strips `#` comments and blank lines, reads
  // `frequency`, `brightness`, `color`. Missing keys keep their defaults.
  Sidecar ReadSidecar(const std::filesystem::path &yaml_path) const {
    Sidecar s;
    std::ifstream f(yaml_path);
    if (!f) {
      RCLCPP_WARN(get_logger(),
                  "no sidecar '%s'; using defaults (frequency %.1f, brightness "
                  "%.1f, no tint)",
                  yaml_path.filename().c_str(), kDefaultFrequency,
                  kDefaultBrightness);
      return s;
    }
    std::string line;
    while (std::getline(f, line)) {
      const auto hash = line.find('#');
      if (hash != std::string::npos) {
        line = line.substr(0, hash);
      }
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      auto trim = [](std::string v) {
        const auto b = v.find_first_not_of(" \t\r\n\"'");
        const auto e = v.find_last_not_of(" \t\r\n\"'");
        return b == std::string::npos ? std::string() : v.substr(b, e - b + 1);
      };
      const std::string key = trim(line.substr(0, colon));
      const std::string val = trim(line.substr(colon + 1));
      if (val.empty()) {
        continue;
      }
      try {
        if (key == "frequency") {
          s.frequency = std::stod(val);
        } else if (key == "brightness") {
          s.brightness = std::clamp(std::stod(val), 0.0, 1.0);
        } else if (key == "color" && val.size() == 7 && val[0] == '#') {
          s.color[0] = static_cast<std::uint8_t>(std::stoi(val.substr(1, 2), nullptr, 16));
          s.color[1] = static_cast<std::uint8_t>(std::stoi(val.substr(3, 2), nullptr, 16));
          s.color[2] = static_cast<std::uint8_t>(std::stoi(val.substr(5, 2), nullptr, 16));
          s.has_color = true;
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "bad value for '%s' in %s: %s", key.c_str(),
                    yaml_path.filename().c_str(), e.what());
      }
    }
    return s;
  }

  // Decode one PNG into led_count-wide rgb8 frames, applying brightness + tint.
  // Wider rows are cropped to the first led_count columns; narrower rows are
  // padded black. Returns false (and warns) on a decode failure.
  bool LoadPng(const std::filesystem::path &png_path, const Sidecar &s,
               Animation &out) const {
    int w = 0, h = 0, channels = 0;
    unsigned char *data = stbi_load(png_path.string().c_str(), &w, &h, &channels, 3);
    if (data == nullptr) {
      RCLCPP_WARN(get_logger(), "failed to decode '%s': %s",
                  png_path.filename().c_str(), stbi_failure_reason());
      return false;
    }
    out.frames.assign(h, std::vector<std::uint8_t>(led_count_ * 3, 0));
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < led_count_ && x < w; ++x) {
        const unsigned char *src = data + (static_cast<std::size_t>(y) * w + x) * 3;
        std::uint8_t rgb[3] = {src[0], src[1], src[2]};
        if (s.has_color) {
          // Grayscale (luma) then tint, mirroring the UGV ImageAnimation
          // color pipeline, so a white-on-black PNG can be recoloured.
          const double luma = (0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]) / 255.0;
          for (int c = 0; c < 3; ++c) {
            rgb[c] = static_cast<std::uint8_t>(luma * s.color[c]);
          }
        }
        std::uint8_t *dst = out.frames[y].data() + x * 3;
        for (int c = 0; c < 3; ++c) {
          dst[c] = static_cast<std::uint8_t>(rgb[c] * s.brightness);
        }
      }
    }
    stbi_image_free(data);
    return true;
  }

  void LoadAnimations(const std::string &user_dir) {
    std::vector<std::filesystem::path> dirs;
    try {
      dirs.emplace_back(
          std::filesystem::path(
              ament_index_cpp::get_package_share_directory("rosbot_utils")) /
          "animations");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "cannot locate shipped animations: %s", e.what());
    }
    if (!user_dir.empty()) {
      dirs.emplace_back(user_dir); // scanned second → overrides shipped by name
    }

    for (const auto &dir : dirs) {
      std::error_code ec;
      if (!std::filesystem::is_directory(dir, ec)) {
        continue;
      }
      for (const auto &entry : std::filesystem::directory_iterator(dir, ec)) {
        if (entry.path().extension() != ".png") {
          continue;
        }
        const std::string name = entry.path().stem().string();
        if (name == kReservedNone) {
          RCLCPP_WARN(get_logger(),
                      "ignoring '%s.png' — 'none' is a reserved name", name.c_str());
          continue;
        }
        const Sidecar sc = ReadSidecar(entry.path().parent_path() / (name + ".yaml"));
        Animation anim;
        anim.frequency = (sc.frequency > 0.0) ? sc.frequency : kDefaultFrequency;
        if (anim.frequency <= 1.0) {
          RCLCPP_WARN(get_logger(),
                      "animation '%s' frequency %.2f Hz <= 1 Hz — the firmware "
                      "idle animation may flicker through",
                      name.c_str(), anim.frequency);
        }
        if (!LoadPng(entry.path(), sc, anim) || anim.frames.empty()) {
          continue;
        }
        animations_[name] = std::move(anim); // user dir overrides shipped
      }
    }
    RCLCPP_INFO(get_logger(), "loaded %zu animation(s): %s", animations_.size(),
                AvailableNames().c_str());
  }

  std::string AvailableNames() const {
    std::string out = kReservedNone;
    for (const auto &[name, _] : animations_) {
      out += ", " + name;
    }
    return out;
  }

  // Switch the active animation. `none` cancels publishing (firmware idle);
  // any loaded name (re)starts a timer at that animation's frequency.
  void Activate(const std::string &name) {
    active_ = name;
    frame_index_ = 0;
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    if (name == kReservedNone) {
      RCLCPP_INFO(get_logger(), "animation: none (publishing stopped)");
      return;
    }
    const auto &anim = animations_.at(name);
    const auto period = std::chrono::duration<double>(1.0 / anim.frequency);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { Tick(); });
    RCLCPP_INFO(get_logger(), "animation: %s (%.1f Hz, %zu frame(s))",
                name.c_str(), anim.frequency, anim.frames.size());
  }

  void Tick() {
    const auto it = animations_.find(active_);
    if (it == animations_.end()) {
      return;
    }
    const auto &frames = it->second.frames;
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.height = 1;
    msg.width = led_count_;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = led_count_ * 3;
    msg.data = frames[frame_index_];
    publisher_->publish(msg);
    frame_index_ = (frame_index_ + 1) % frames.size();
  }

  int led_count_ = 18;
  std::map<std::string, Animation> animations_;
  std::string active_ = kReservedNone;
  std::size_t frame_index_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnimationPublisher>());
  rclcpp::shutdown();
  return 0;
}
