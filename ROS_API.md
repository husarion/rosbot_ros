# ROSbot Series - Software

Detailed information about content of rosbot package for ROS2.

## ROS API

### Available Nodes

[controller_manager/controller_manager]: https://github.com/ros-controls/ros2_control/blob/master/controller_manager
[diff_drive_controller/diff_drive_controller]: https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller
[gz_ros2_control/gz_ros2_control]: https://github.com/ros-controls/gz_ros2_control
[imu_sensor_broadcaster/imu_sensor_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster
[joint_state_broadcaster/joint_state_broadcaster]: https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster
[laser_filters/scan_to_scan_filter_chain]: https://github.com/ros-perception/laser_filters/blob/ros2/src/scan_to_scan_filter_chain.cpp
[micro_ros_agent/micro_ros_agent]: https://github.com/micro-ROS/micro-ROS-Agent
[robot_localization/ekf_node]: https://github.com/cra-ros-pkg/robot_localization
[robot_state_publisher/robot_state_publisher]: https://github.com/ros/robot_state_publisher
[rosbot_hardware_interfaces/rosbot_imu_sensor]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_imu_sensor.cpp
[rosbot_hardware_interfaces/rosbot_system]: https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_system.cpp
[ros_gz_bridge/parameter_bridge]: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge

| 🤖  | 🖥️  | NODE                          | DESCRIPTION                                                                                                                                                                                                                                                                                                                                         |
| --- | --- | ----------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ✅  | **`controller_manager`**      | Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. <br /> _[controller_manager/controller_manager]_                             |
| ✅  | ✅  | **`drive_controller`**  | The controller managing a mobile robot with a differential or omni drive (mecanum wheels). Converts speed commands for the robot body to wheel commands for the base. It also calculates odometry based on hardware feedback and shares it.`DiffDriveController` or `MecanumDriveController` <br /> _[diff_drive_controller/diff_drive_controller]_ |
| ✅  | ✅  | **`ekf_node`**                | Used to fuse wheel odometry and IMU data. Parameters are defined in `rosbot_bringup/config/ekf.yaml` <br /> _[robot_localization/ekf_node]_                                                                                                                                                                                                         |
| ❌  | ✅  | **`/gz_bridge`**              | Transmits Gazebo simulation data to the ROS layer <br /> _[ros_gz_bridge/parameter_bridge]_                                                                                                                                                                                                                                         |
| ❌  | ✅  | **`gz_ros_control`**         | Responsible for integrating the ros2_control controller architecture with the Gazebo simulator. <br /> _[gz_ros2_control/gz_ros2_control]_                                                                                                                                                                                                                                         |
| ✅  | ✅  | **`imu_broadcaster`**         | The broadcaster to publish readings of IMU sensors <br /> _[imu_sensor_broadcaster/imu_sensor_broadcaster]_                                                                                                                                                                                                                                         |
| ✅  | ❌  | **`imu_sensor_node`**         | The node responsible for subscriptions to IMU data from the hardware <br /> _[rosbot_hardware_interfaces/rosbot_imu_sensor]_                                                                                                                                                                                                                        |
| ✅  | ✅  | **`joint_state_broadcaster`** | The broadcaster reads all state interfaces and reports them on specific topics <br /> _[joint_state_broadcaster/joint_state_broadcaster]_                                                                                                                                                                                                           |
| ✅  | ✅  | **`laser_filter`**            | This is a filter that removes points in a laser scan inside of a cartesian box <br /> _[laser_filters/scan_to_scan_filter_chain]_                                                                                                                                                                                                                   |
| ✅  | ✅  | **`robot_state_publisher`**   | Uses the URDF specified by the parameter robot\*description and the joint positions from the topic joint\*states to calculate the forward kinematics of the robot and publish the results using tf <br /> _[robot_state_publisher/robot_state_publisher]_                                                                                             |
| ✅  | ❌  | **`rosbot_system_node`**      | The node communicating with the hardware responsible for receiving and sending data related to engine control <br /> _[rosbot_hardware_interfaces/rosbot_system]_                                                                                                                                                                                   |
| ❌  | ✅  | **`rosbot_gz_bridge`**        | Transmits data about the robot between the Gazebo simulator and ROS. <br /> _[ros_gz_bridge/parameter_bridge]_ |
| ✅  | ❌  | **`/stm32_node`**             | Node responsible for communication with hardware. <br /> _[micro_ros_agent/micro_ros_agent]_                                                                                                                                                                                                                      |

### Available Topics

[control_msgs/DynamicJointState]: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/DynamicJointState.msg
[diagnostic_msgs/DiagnosticArray]: https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html
[geometry_msgs/PoseWithCovarianceStamped]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovarianceStamped.html
[geometry_msgs/Twist]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html
[nav_msgs/Odometry]: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html
[sensor_msgs/Imu]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
[sensor_msgs/JointState]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html
[sensor_msgs/LaserScan]: https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
[std_msgs/Float32MultiArray]: https://docs.ros2.org/foxy/api/std_msgs/msg/Float32MultiArray.html
[std_msgs/String]: https://docs.ros2.org/foxy/api/std_msgs/msg/String.html
[tf2_msgs/TFMessage]: https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html

| 🤖  | 🖥️  | TOPIC                                          | DESCRIPTION                                                                                                                   |
| --- | --- | ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ❌  | **`/battery_state`**                           | Provides information about the state of the battery. <br /> _[sensor_msgs/BatteryState]_                                      |
| ✅  | ✅  | **`cmd_vel`**                                  | Sends velocity commands for controlling robot motion. <br /> _[geometry_msgs/Twist]_                                          |
| ✅  | ✅  | **`diagnostics`**                              | Contains diagnostic information about the robot's systems. <br /> _[diagnostic_msgs/DiagnosticArray]_                         |
| ✅  | ✅  | **`dynamic_joint_states`**                     | Publishes information about the dynamic state of joints. <br /> _[control_msgs/DynamicJointState]_                            |
| ✅  | ✅  | **`imu_broadcaster/imu`**                      | Broadcasts IMU (Inertial Measurement Unit) data. <br /> _[sensor_msgs/Imu]_                                                   |
| ✅  | ✅  | **`joint_states`**                             | Publishes information about the state of robot joints. <br /> _[sensor_msgs/JointState]_                                      |
| ✅  | ✅  | **`odometry/filtered`**                        | Publishes filtered odometry data. <br /> _[nav_msgs/Odometry]_                                                                |
| ✅  | ✅  | **`odometry/wheels`**              | Provides odometry data from the base controller of the ROSbot XL. <br /> _[nav_msgs/Odometry]_                                |
| ✅  | ✅  | **`robot_description`**                        | Publishes the robot's description. <br /> _[std_msgs/String]_                                                                 |
| ✅  | ✅  | **`scan`**                                     | Publishes raw laser scan data. <br /> _[sensor_msgs/LaserScan]_                                                               |
| ✅  | ✅  | **`scan_filtered`**                            | Publishes filtered laser scan data. <br /> _[sensor_msgs/LaserScan]_                                                          |
| ✅  | ✅  | **`set_pose`**                                 | Sets the robot's pose with covariance. <br /> _[geometry_msgs/PoseWithCovarianceStamped]_                                     |
| ✅  | ✅  | **`tf`**                                       | Publishes transformations between coordinate frames over time. <br /> _[tf2_msgs/TFMessage]_                                  |
| ✅  | ✅  | **`tf_static`**                                | Publishes static transformations between coordinate frames. <br /> _[tf2_msgs/TFMessage]_                                     |

**Hidden topic:**

| TOPIC                    | DESCRIPTION                                                         |
| ------------------------ | ------------------------------------------------------------------- |
| **`/_imu/data_raw`**     | raw data image from imu sensor <br /> _[sensor_msgs/Imu]_           |
| **`/_motors_cmd`**       | desired speed on each wheel <br /> _[std_msgs/Float32MultiArray]_   |
| **`/_motors_responses`** | raw data readings from each wheel <br /> _[sensor_msgs/JointState]_ |

## Package Description

### `rosbot`

Metapackage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used.

### `rosbot_bringup`

The main package responsible for running the physical robot.

**Available Launch Files:**

- `bringup.launch.py` - is responsible for communicating with firmware and activating all logic related to the robot's movement and processing of sensory data.
- `microros.launch.py` - establishes connection with the hardware using microROS agent.

### `rosbot_controller`

ROS2 hardware controller for ROSbot. It manages inputs and outputs data from ROS2 control, forwarding it via ROS topics to be read by microROS. The controller.launch.py file loads the robot model defined in rosbot_description along with ROS2 control dependencies from [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces).

**Available Launch Files:**

- `controller.launch.py` - starts controllers related to ros2_control responsible for driving, communication with imu and joint_states publications

### `rosbot_description`

URDF model used for both simulation and as a source of transforms on physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

**Available Launch Files:**

- `load_urdf.launch.py` - starts controllers related to ros2_control responsible for driving, communication with imu and joint_states publications
- `rviz.launch.py` - Launches the ready-made RViz configuration

**Main Description Files:**

- `rosbot.urdf.xacro` - Final configuration of ROSbot.
- `rosbot_xl.urdf.xacro` - Final configuration of ROSbot XL.

### `rosbot_gazebo`

Launch files for Gazebo working with ROS2 control.

**Available Launch Files:**

- `simulations.launch.py` - Runs simulations with a defined robot and all sensors on it.
- `spawn_robot.launch.py` - Allow to spawn new robot in already running simulation.

### `rosbot_localization`

A package related to the logic responsible for performing sensor fusion.

**Available Launch Files:**

- `ekf.launch.py` - Runs ekf filter which fuse wheel odometry with imu data.

### `rosbot_utils`

A package containing auxiliary filters that integrate simple external packages.

**Available Launch Files:**

- `laser_filter.launch.py` - launch laser filter responsible for filtering out the laser scan points located inside the robot base.
