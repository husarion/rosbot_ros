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

// Guards the contract between rosbot_hardware_interfaces.xml and the
// PLUGINLIB_EXPORT_CLASS macros in src/*.cpp. A mismatch (class rename, ABI
// drift, missing macro) only fails at runtime when controller_manager tries
// to load the plugin on HW; this test fails the build instead.

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(PluginRegistration, RosbotSystemLoadable) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  ASSERT_NO_THROW({
    auto plugin =
        loader.createSharedInstance("rosbot_hardware_interfaces/RosbotSystem");
    ASSERT_NE(plugin, nullptr);
  });
}

TEST(PluginRegistration, RosbotImuSensorLoadable) {
  pluginlib::ClassLoader<hardware_interface::SensorInterface> loader(
      "hardware_interface", "hardware_interface::SensorInterface");
  ASSERT_NO_THROW({
    auto plugin = loader.createSharedInstance(
        "rosbot_hardware_interfaces/RosbotImuSensor");
    ASSERT_NE(plugin, nullptr);
  });
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Defense in depth: keep DDS off the network even if the test runs outside
  // CMake's ENV block (eg. invoked directly via ctest binary). Pairs with the
  // ENV setting in CMakeLists.txt.
  setenv("ROS_DOMAIN_ID", "87", 1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  // The plugin constructors instantiate rclcpp publishers/subscribers, which
  // require a valid rclcpp context. Initialize it here and shutdown on exit.
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
