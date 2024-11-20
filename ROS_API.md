Use `bringup.launch.py` from `rosbot_bringup` to start all base functionalities for ROSbot 2, 2 PRO, 2R. It consists of following parts:

- `ekf_node` from `robot_localization`, it is used to fuse wheel odometry and IMU data. Parameters are defined in `ekf.yaml` in `rosbot_bringup/config`. It subscribes to `/rosbot_base_controller/odom` and `/imu_broadcaster/imu` published by ros2 controllers and publishes fused odometry on `/odometry/filtered` topic

  **Subscribes**
  - `/rosbot_base_controller/odom` (_nav_msgs/Odometry_)
  - `/imu_broadcaster/imu` (_sensor_msgs/Imu_)

  **Publishes**
  - `/tf` (_tf2_msgs/TFMessage_) - `base_link`->`odom` transform
  - `/odometry/filtered` (_nav_msgs/Odometry_)

Use `controller.launch.py` from `rosbot_controller`, it loads robot model defined in `rosbot_description` as well as ros2 control [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces). It also starts controllers:

- `joint_state_broadcaster`
- `rosbot_base_controller`
- `imu_broadcaster`

  **Subscribes**
  - `/cmd_vel` (_geometry_msgs/Twist_)
  - `/_motors_responses` (_sensor_msgs/JointState_)
  - `/_imu/data_raw` (_sensor_msgs/Imu_)

  **Publishes**
  - `/tf` (_tf2_msgs/TFMessage_)
  - `/tf_static` (_tf2_msgs/TFMessage_)
  - `/_motors_cmd` (_std_msgs/Float32MultiArray_)
  - `/rosbot_base_controller/odom` (_nav_msgs/Odometry_)
  - `/imu_broadcaster/imu` (_sensor_msgs/Imu_)

Use `simulation.launch.py` from `rosbot_gazebo` to start all base functionalities for ROSbot 2, 2 PRO, 2R in the Gazebo simulator.
If you want to spawn multiple robots use `simulation.launch.py` with the `robots` argument e. g.:

```bash
ros2 launch rosbot_gazebo simulation.launch.py robots:='robot1={x: 0.0, y: -1.0}; robot2={x: 1.0, y: -1.0}; robot3={x: 2.0, y: -1.0}'
```

If you want to use your own world add to the world's sdf file gazebo sensors plugins inside any `<model>` tag:

```xml
<plugin filename="ignition-gazebo-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors"/>
```

> **Warning**
> The distance sensors' topics types from Gazebo simulation mismatch with the real ones. The range sensors are not implemented yet in the Gazebo Ignition (for more information look [here](https://github.com/gazebosim/gz-sensors/issues/19)). The real type is [sensor_msgs/msg/Range](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg) but simulated [sensor_msgs/msg/LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg). The first value of the `ranges` in [sensor_msgs/msg/LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg) is the `range` field of [sensor_msgs/msg/Range](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg).
