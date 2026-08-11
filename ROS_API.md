# ROSbot Series - Software

Detailed information about content of rosbot package for ROS2.

## Control

### Gamepad

After running the ROSbot XL Manipulation Package, you should be able to control the manipulator. The easiest way to move the manipulator is to connect a gamepad and steer the robot. The graphic below shows how to steer the manipulator using a gamepad.

![gamepad_rosbot](.docs/gamepad_rosbot.drawio.png)

Drive controls are defined in [`rosbot_joy/config/config.yaml`](rosbot_joy/config/config.yaml) and published to `manual/cmd_vel`, the highest-priority input of [velocity command arbitration](#velocity-command-arbitration) — a held gamepad always overrides navigation. The manipulator gamepad mappings (XL only) are hardcoded in [`rosbot_moveit/src/joy2servo.cpp`](rosbot_moveit/src/joy2servo.cpp).

## ROS API

### Namespace policy

Everything below is published under `/<namespace>/` when the `namespace`
launch arg (or `ROBOT_NAMESPACE` env) is set. Intentional globals:

- `/tf`, `/tf_static` — bridged via [tf_namespace_bridge](https://github.com/husarion/tf_namespace_bridge).
- `/parameter_events`, `/rosout` — ROS 2 infra.
- `/clock` — sim only.
- `/asset_providers` — `husarion_asset_server`'s `AssetProviderInfo` announcement; global by design so a router/bridge discovers every provider across every robot namespace on one topic.

Hard-coded, no runtime opt-out. HW uses `push_ros_namespace`, sim uses URDF
`<remapping>` for the `controller_manager` surface — see
[Namespacing and multirobot](ARCHITECTURE.md#5-namespacing-and-multirobot).
Enforced by
[test_namespace_isolation.py](rosbot_bringup/test/test_namespace_isolation.py).

### Velocity command arbitration

Several sources may want to drive the robot at once — a gamepad, a navigation
stack, an ad-hoc script. `twist_mux_controller` picks one by priority and feeds
it into the drive controller through ros2_control reference interfaces, so the
arbitration happens inside the 100 Hz control loop rather than over topics.

| Input | Topic | Priority | Typical publisher |
| --- | --- | --- | --- |
| `manual` | `manual/cmd_vel` | 100 | gamepad (`rosbot_joy`), keyboard teleop |
| `autonomous` | `autonomous/cmd_vel` | 10 | nav2 |
| `unknown` | `cmd_vel` | 1 | anything else |

The highest-priority input that published within the last **0.2 s** wins; once
it goes quiet control falls through to the next one down. So holding the
gamepad overrides navigation, and releasing it hands control back automatically
— no service call, no mode switch.

`twist_mux_controller/source` reports which input is currently in charge.

Because the mux claims the drive controller's reference interfaces, the drive
controller runs in *chained mode* and no longer subscribes to its own
`~/cmd_vel`. Publishing to `cmd_vel` still works — it is simply the
lowest-priority input now. Priorities and timeouts live in
[`rosbot_controller/config/<model>/controllers.yaml`](rosbot_controller/config/).

### Available Nodes

[controller_manager/controller_manager]: https://github.com/ros-controls/ros2_control/blob/master/controller_manager
[diff_drive_controller/diff_drive_controller]: https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller
[gz_ros2_control/gz_ros2_control]: https://github.com/ros-controls/gz_ros2_control
[husarion_asset_server/asset_server]: https://github.com/husarion/husarion_asset_server
[imu_sensor_broadcaster/imu_sensor_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster
[joint_state_broadcaster/joint_state_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster
[robot_localization/ekf_node]: https://github.com/cra-ros-pkg/robot_localization
[robot_state_publisher/robot_state_publisher]: https://github.com/ros/robot_state_publisher
[rosbot_hardware_interfaces/rosbot_imu_sensor]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_imu_sensor.cpp
[rosbot_hardware_interfaces/rosbot_system]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_system.cpp
[ros_gz_bridge/parameter_bridge]: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge

| 🤖  | 🖥️  | NODE                          | DESCRIPTION                                                                                                                                                                                                                                                                                                                                         |
| --- | --- | ----------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ✅  | **`controller_manager`**      | Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. <br /> *[controller_manager/controller_manager]*                             |
| ✅  | ✅  | **`differential_drive_controller`** / **`mecanum_drive_controller`**  | The controller managing a mobile robot with a differential or omni drive (mecanum wheels). Converts speed commands for the robot body to wheel commands for the base. It also calculates odometry based on hardware feedback and shares it.`DiffDriveController` or `MecanumDriveController` <br /> *[diff_drive_controller/diff_drive_controller]* |
| ✅  | ✅  | **`ekf_node`**                | Used to fuse wheel odometry and IMU data. Parameters are defined in `rosbot_localization/config/config.yaml` <br /> *[robot_localization/ekf_node]*                                                                                                                                                                                                         |
| ❌  | ✅  | **`/gz_bridge`**              | Transmits Gazebo simulation data to the ROS layer <br /> *[ros_gz_bridge/parameter_bridge]*                                                                                                                                                                                                                                         |
| ❌  | ✅  | **`gz_ros_control`**         | Responsible for integrating the ros2_control controller architecture with the Gazebo simulator. <br /> *[gz_ros2_control/gz_ros2_control]*                                                                                                                                                                                                                                         |
| ✅  | ❌  | **`husarion_asset_server`**  | Serves this robot's `package://` meshes/URDF resources over a `get_asset` service; auto-derives owned packages from the co-located `robot_description`. Toggle with the `asset_server` launch arg. <br /> *[husarion_asset_server/asset_server]*                                                                                                                                  |
| ✅  | ✅  | **`imu_broadcaster`**         | The broadcaster to publish readings of IMU sensors <br /> *[imu_sensor_broadcaster/imu_sensor_broadcaster]*                                                                                                                                                                                                                                         |
| ✅  | ❌  | **`imu_sensor_node`**         | The node responsible for subscriptions to IMU data from the hardware <br /> *[rosbot_hardware_interfaces/rosbot_imu_sensor]*                                                                                                                                                                                                                        |
| ✅  | ✅  | **`joint_state_broadcaster`** | The broadcaster reads all state interfaces and reports them on specific topics <br /> *[joint_state_broadcaster/joint_state_broadcaster]*                                                                                                                                                                                                           |
| ✅  | ✅  | **`robot_state_publisher`**   | Uses the URDF specified by the parameter robot\*description and the joint positions from the topic joint\*states to calculate the forward kinematics of the robot and publish the results using tf <br /> *[robot_state_publisher/robot_state_publisher]*                                                                                             |
| ✅  | ❌  | **`rosbot_system_node`**      | The node communicating with the hardware responsible for receiving and sending data related to engine control <br /> *[rosbot_hardware_interfaces/rosbot_system]*                                                                                                                                                                                   |
| ❌  | ✅  | **`rosbot_gz_bridge`**        | Transmits data about the robot between the Gazebo simulator and ROS. <br /> *[ros_gz_bridge/parameter_bridge]* |
| ✅  | ❌  | **`rosbot_mcu`**             | Microcontroller unit (MCU) communication node. Default: `rosbot_mavlink_bridge` (MAVLink). With `backend:=microros` it is replaced by the `micro_ros_agent` node, which speaks XRCE-DDS to the MCU instead. <br /> *[rosbot_mavlink_bridge/rosbot_mavlink_bridge]*                                                                                                                                                                                                                      |
| ✅  | ✅  | **`twist_mux_controller`**   | Chainable controller arbitrating velocity commands from several sources by priority and forwarding the winner straight into the drive controller's reference interfaces — the arbitration runs inside the 100 Hz control loop, not over topics. See [Velocity command arbitration](#velocity-command-arbitration). <br /> *[twist_mux_controller/TwistMuxController]* |

### Available Topics

[control_msgs/DynamicJointState]: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/DynamicJointState.msg
[diagnostic_msgs/DiagnosticArray]: https://docs.ros.org/en/jazzy/p/diagnostic_msgs/msg/DiagnosticArray.html
[geometry_msgs/PoseWithCovarianceStamped]: https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/PoseWithCovarianceStamped.html
[geometry_msgs/TwistStamped]: https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/TwistStamped.html
[nav_msgs/Odometry]: https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html
[sensor_msgs/Imu]: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html
[sensor_msgs/JointState]: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html
[sensor_msgs/Joy]: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Joy.html
[sensor_msgs/LaserScan]: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html
[std_msgs/String]: https://docs.ros.org/en/jazzy/p/std_msgs/msg/String.html
[tf2_msgs/TFMessage]: https://docs.ros.org/en/jazzy/p/tf2_msgs/msg/TFMessage.html

| 🤖  | 🖥️  | TOPIC                                          | DESCRIPTION                                                                                                                   |
| --- | --- | ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ✅  | **`autonomous/cmd_vel`**                       | Velocity commands from an autonomy stack (nav2). Priority 10 — yields to `manual/cmd_vel`. <br /> *[geometry_msgs/TwistStamped]* |
| ✅  | ✅  | **`cmd_vel`**                                  | Velocity commands from an unclassified source. Priority 1 — the lowest, kept for backwards compatibility. <br /> *[geometry_msgs/TwistStamped]* |
| ✅  | ✅  | **`diagnostics`**                              | Contains diagnostic information about the robot's systems. <br /> *[diagnostic_msgs/DiagnosticArray]*                         |
| ✅  | ✅  | **`dynamic_joint_states`**                     | Publishes information about the dynamic state of joints. <br /> *[control_msgs/DynamicJointState]*                            |
| ✅  | ✅  | **`imu/data`**                      | Broadcasts IMU (Inertial Measurement Unit) data. <br /> *[sensor_msgs/Imu]*                                                   |
| ✅  | ✅  | **`joint_states`**                             | Publishes information about the state of robot joints. On hardware the `effort` field carries wheel motor torque (measured on ROSbot XL rev 1.2, back-EMF estimate otherwise); in simulation `effort` is `NaN`. <br /> *[sensor_msgs/JointState]*                                      |
| ✅  | ✅  | **`joy`**                             | Publishes joystick input data. <br /> *[sensor_msgs/Joy]*                                      |
| ✅  | ✅  | **`manual/cmd_vel`**                           | Velocity commands from a human operator (gamepad, keyboard teleop). Priority 100 — the highest, so it overrides autonomy. <br /> *[geometry_msgs/TwistStamped]* |
| ✅  | ✅  | **`odometry/filtered`**                        | Publishes filtered odometry data. <br /> *[nav_msgs/Odometry]*                                                                |
| ✅  | ✅  | **`odometry/wheels`**              | Provides odometry data from the base controller of the ROSbot XL. <br /> *[nav_msgs/Odometry]*                                |
| ✅  | ✅  | **`robot_description`**                        | Publishes the robot's description. <br /> *[std_msgs/String]*                                                                 |
| ✅  | ✅  | **`scan`**                                     | Publishes raw laser scan data. <br /> *[sensor_msgs/LaserScan]*                                                               |
| ✅  | ✅  | **`set_pose`**                                 | Changes the robot's `odometry/filtered` pose. <br /> *[geometry_msgs/PoseWithCovarianceStamped]*                                     |
| ✅  | ✅  | **`tf`**                                       | Publishes transformations between coordinate frames over time. <br /> *[tf2_msgs/TFMessage]*                                  |
| ✅  | ✅  | **`tf_static`**                                | Publishes static transformations between coordinate frames. <br /> *[tf2_msgs/TFMessage]*                                     |
| ✅  | ✅  | **`twist_mux_controller/source`**              | Name of the input currently driving the robot: `manual`, `autonomous`, `unknown` or `not_published`. <br /> *[std_msgs/String]* |

There are also additional topics related with the ROSbot firmware. For more information about them, please refer to the [ROSbot Firmware documentation](https://github.com/husarion/rosbot-firmware/blob/jazzy/ROS_API.md).

### Available Services

[std_srvs/SetBool]: https://docs.ros.org/en/jazzy/p/std_srvs/srv/SetBool.html
[husarion_asset_msgs/srv/GetAsset]: https://github.com/husarion/husarion_asset_msgs

| 🤖  | 🖥️  | SERVICE                | DESCRIPTION                                                                                                                                                                |
| --- | --- | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ❌  | **`husarion_asset_server/get_asset`** | Resolves a `package://PKG/REL` URI to bytes (ranged fetch) for the description packages `husarion_asset_server` owns. <br /> *[husarion_asset_msgs/srv/GetAsset]* |
| ✅  | ❌  | **`led_strip/enable`** | ROSbot XL only. Enables (`data: true`) or disables (`data: false`) the LED strip animation. While disabled the `animation_publisher` node neither computes nor publishes the `led_strip` image, and the firmware idle animation shows instead. The `current_animation` parameter is remembered across the gate, so enabling resumes whatever was selected. <br /> *[std_srvs/SetBool]* |

## Packages

One-line purpose per package; full detail (launch flows, internals) in
[ARCHITECTURE.md](ARCHITECTURE.md#2-packages).

| Package | Description |
| --- | --- |
| [`rosbot`](rosbot/) | Meta-package — pins sibling repos via `*.repos`, no code. |
| [`rosbot_bringup`](rosbot_bringup/) | Hardware entry point: per-model bringup + MCU backend (MAVLink default / micro-ROS). *Local-only.* |
| [`rosbot_controller`](rosbot_controller/) | ros2_control setup — spawns drive, IMU and joint-state controllers (plus the manipulator on XL). |
| [`rosbot_description`](rosbot_description/) | URDF/xacro for hardware and simulation, robot configurations, `robot_state_publisher`. |
| [`rosbot_gazebo`](rosbot_gazebo/) | Gazebo simulation launch and robot spawning. *Local-only.* |
| [`rosbot_hardware_interfaces`](rosbot_hardware_interfaces/) | C++ ros2_control plugins (`RosbotSystem`, `RosbotImuSensor`) — the firmware ABI. |
| [`rosbot_joy`](rosbot_joy/) | Joystick teleop for driving (`joy_node` + `teleop_twist_joy`). |
| [`rosbot_localization`](rosbot_localization/) | EKF fusing wheel odometry + IMU → `odometry/filtered`. |
| [`rosbot_moveit`](rosbot_moveit/) | MoveIt manipulation for the OpenMANIPULATOR-X (XL only) — see [MANIPULATOR.md](MANIPULATOR.md). |
| [`rosbot_utils`](rosbot_utils/) | Utilities: firmware flashing, robot configuration, udev rules, battery alert, LED strip. |
