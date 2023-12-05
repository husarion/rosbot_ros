// Copyright 2023 Husarion sp. z o.o.
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

#include "fstream"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define LOOP_PERIOD 2s
#define MSG_VALID_TIME 5s

std::chrono::steady_clock::time_point last_msg_time;

void write_health_status(const std::string &status) {
  std::ofstream healthFile("/var/tmp/health_status.txt");
  healthFile << status;
}

void msg_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_msg_time = std::chrono::steady_clock::now();
}

void healthy_check() {
  std::chrono::steady_clock::time_point current_time =
      std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - last_msg_time;
  bool is_msg_valid = elapsed_time.count() < MSG_VALID_TIME.count();

  if (is_msg_valid) {
    write_health_status("healthy");
  } else {
    write_health_status("unhealthy");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", rclcpp::SensorDataQoS(), msg_callback);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    healthy_check();
    std::this_thread::sleep_for(LOOP_PERIOD);
  }

  return 0;
}
