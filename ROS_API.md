# ROSbot - Software

Detailed information about content of rosbot package for ROS2.

## Package Description

### `rosbot`

Metapackage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used.

### `rosbot_bringup`

Package that contains launch, which starts all base functionalities with the microros agent. Also configs for `robot_localization` and `laser_filters` are defined there.

**Available Launch Files:**

- `bringup.launch.py` - is responsible for communicating with firmware and activating all logic related to the robot's movement and processing of sensory data.
- `microros.launch.py` - establishes connection with the firmware.

### `rosbot_controller`

ROS2 hardware controller for ROSbot. It manages inputs and outputs data from ROS2 control, forwarding it via ROS topics to be read by microROS. The controller.launch.py file loads the robot model defined in rosbot_description along with ROS2 control dependencies from [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces).

### `rosbot_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

Available models:

| MODEL         | DESCRIPTION                                                                                                                                                                                  |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rosbot`      | Final configuration of rosbot with ability to attach external hardware.                                                                                                                      |
| `rosbot_base` | Base of rosbot prepared to be included into preexisting configuration. Meant to be compatible with concept of ROS Industrial ability for manipulators to have interchangeable end effectors. |

### `rosbot_gazebo`

Launch files for Ignition Gazebo working with ROS2 control.

**Available Launch Files:**

- `simulations.launch.py` - running a rosbot in Gazebo simulator and simulate all specified sensors.

### `rosbot_utils`

This package contains the stable firmware version with the flash script.

## ROS API

### Available Nodes

[controller_manager/controller_manager]: https://github.com/ros-controls/ros2_control/blob/master/controller_manager
[diff_drive_controller/diff_drive_controller]: https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller
[imu_sensor_broadcaster/imu_sensor_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster
[joint_state_broadcaster/joint_state_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster
[laser_filters/scan_to_scan_filter_chain]: https://github.com/ros-perception/laser_filters/blob/ros2/src/scan_to_scan_filter_chain.cpp
[micro_ros_agent/micro_ros_agent]: https://github.com/micro-ROS/micro-ROS-Agent
[robot_localization/ekf_node]: https://github.com/cra-ros-pkg/robot_localization
[robot_state_publisher/robot_state_publisher]: https://github.com/ros/robot_state_publisher
[rosbot_hardware_interfaces/rosbot_imu_sensor]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_imu_sensor.cpp
[rosbot_hardware_interfaces/rosbot_system]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_system.cpp

| NODE                              | DESCRIPTION                                                                                                                                                                                                                                                                                                                                           |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`~/controller_manager`**        | Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. <br /> _[controller_manager/controller_manager][]_                             |
| **`~/ekf_filter_node`**           | Used to fuse wheel odometry and IMU data. Parameters are defined in `rosbot_bringup/config/ekf.yaml` <br /> _[robot_localization/ekf_node][]_                                                                                                                                                                                                         |
| **`~/imu_broadcaster`**           | The broadcaster to publish readings of IMU sensors <br /> _[imu_sensor_broadcaster/imu_sensor_broadcaster][]_                                                                                                                                                                                                                                         |
| **`~/imu_sensor_node`**           | The node responsible for subscriptions to IMU data from the hardware <br /> _[rosbot_hardware_interfaces/rosbot_imu_sensor][]_                                                                                                                                                                                                                        |
| **`~/joint_state_broadcaster`**   | The broadcaster reads all state interfaces and reports them on specific topics <br /> _[joint_state_broadcaster/joint_state_broadcaster][]_                                                                                                                                                                                                           |
| **`~/laser_scan_box_filter`**     | This is a filter that removes points in a laser scan inside of a cartesian box <br /> _[laser_filters/scan_to_scan_filter_chain][]_                                                                                                                                                                                                                   |
| **`~/robot_state_publisher`**     | Uses the URDF specified by the parameter robot\*description and the joint positions from the topic joint\*states to calculate the forward kinematics of the robot and publish the results via tf <br /> _[robot_state_publisher/robot_state_publisher][]_                                                                                             |
| **`~/rosbot_system_node`**        | The node communicating with the hardware responsible for receiving and sending data related to engine control <br />  _[rosbot_hardware_interfaces/rosbot_system][]_                                                                                                                                                                                  |
| **`~/rosbot_base_controller`**    | The controller managing a mobile robot with a differential or omni drive (mecanum wheels). Converts speed commands for the robot body to wheel commands for the base. It also calculates odometry based on hardware feedback and shares it.`DiffDriveController` or `MecanumDriveController` <br /> _[diff_drive_controller/diff_drive_controller][]_ |
| **`~/scan_to_scan_filter_chain`** | Node which subscribes to `/scan` topic and removes all points that are within the robot's footprint (defined by config `laser_filter.yaml` in `rosbot_bringup` package). Filtered laser scan is then published on `/scan_filtered` topic <br /> _[laser_filters/scan_to_scan_filter_chain][]_                                                         |
| **`/stm32_node`**                 | Node enabling communication with Digital Board, it provides the following interface <br /> _[micro_ros_agent/micro_ros_agent][]_                                                                                                                                                                                                                      |

### Available Topics

[control_msgs/DynamicJointState]: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/DynamicJointState.msg
[diagnostic_msgs/DiagnosticArray]: https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html
[geometry_msgs/PoseWithCovarianceStamped]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovarianceStamped.html
[geometry_msgs/Twist]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html
[lifecycle_msgs/TransitionEvent]: https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/TransitionEvent.html
[nav_msgs/Odometry]: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html
[sensor_msgs/Imu]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
[sensor_msgs/JointState]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html
[sensor_msgs/LaserScan]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
[std_msgs/Float32MultiArray]: https://docs.ros2.org/foxy/api/std_msgs/msg/Float32MultiArray.html
[std_msgs/String]: https://docs.ros2.org/foxy/api/std_msgs/msg/String.html
[tf2_msgs/TFMessage]: https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html

| TOPIC                                            | DESCRIPTION                                                                                                                     |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| **`/battery_state`**                             | provides information about the state of the battery. <br /> _[sensor_msgs/BatteryState][]_                                      |
| **`~/cmd_vel`**                                  | sends velocity commands for controlling robot motion. <br /> _[geometry_msgs/Twist][]_                                          |
| **`/diagnostics`**                               | contains diagnostic information about the robot's systems. <br /> _[diagnostic_msgs/DiagnosticArray][]_                         |
| **`~/dynamic_joint_states`**                     | publishes information about the dynamic state of joints. <br /> _[control_msgs/DynamicJointState][]_                            |
| **`~/imu_broadcaster/imu`**                      | broadcasts IMU (Inertial Measurement Unit) data. <br /> _[sensor_msgs/Imu][]_                                                   |
| **`~/imu_broadcaster/transition_event`**         | signals transition events in the lifecycle of the IMU broadcaster node. <br /> _[lifecycle_msgs/TransitionEvent][]_             |
| **`~/joint_state_broadcaster/transition_event`** | indicates transition events in the lifecycle of the joint state broadcaster node. <br /> _[lifecycle_msgs/TransitionEvent][]_   |
| **`~/joint_states`**                             | publishes information about the state of robot joints. <br /> _[sensor_msgs/JointState][]_                                      |
| **`~/laser_scan_box_filter/transition_event`**   | signals transition events in the lifecycle of the laser scan box filter node. <br /> _[lifecycle_msgs/TransitionEvent][]_       |
| **`~/odometry/filtered`**                        | publishes filtered odometry data. <br /> _[nav_msgs/Odometry][]_                                                                |
| **`~/robot_description`**                        | publishes the robot's description. <br /> _[std_msgs/String][]_                                                                 |
| **`~/rosbot_base_controller/odom`**              | provides odometry data from the base controller of the ROSbot. <br /> _[nav_msgs/Odometry][]_                                |
| **`~/rosbot_base_controller/transition_event`**  | indicates transition events in the lifecycle of the ROSbot base controller node. <br /> _[lifecycle_msgs/TransitionEvent][]_ |
| **`~/scan`**                                     | publishes raw laser scan data. <br /> _[sensor_msgs/LaserScan][]_                                                               |
| **`~/scan_filtered`**                            | publishes filtered laser scan data. <br /> _[sensor_msgs/LaserScan][]_                                                          |
| **`~/set_pose`**                                 | sets the robot's pose with covariance. <br /> _[geometry_msgs/PoseWithCovarianceStamped][]_                                     |
| **`~/tf`**                                       | publishes transformations between coordinate frames over time. <br /> _[tf2_msgs/TFMessage][]_                                  |
| **`~/tf_static`**                                | publishes static transformations between coordinate frames. <br /> _[tf2_msgs/TFMessage][]_                                     |

**Hidden topic:**

| TOPIC                    | DESCRIPTION                                                           |
| ------------------------ | --------------------------------------------------------------------- |
| **`/_imu/data_raw`**     | raw data image from imu sensor <br /> _[sensor_msgs/Imu][]_           |
| **`/_motors_cmd`**       | desired speed on each wheel <br /> _[std_msgs/Float32MultiArray][]_   |
| **`/_motors_responses`** | raw data readings from each wheel <br /> _[sensor_msgs/JointState][]_ |
