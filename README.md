# rosbot_description #

Simulation model for Gazebo integrated with ROS 2.

## Installation. ## 

We assume that you are working on Ubuntu 18.04 and already have installed ROS Dashing. If not, follow the [ROS install guide](https://index.ros.org/doc/ros2/Installation/Dashing/)

Prepare the repository ([more about building packages](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)):
```
mkdir ~/ros2_workspace
mkdir ~/ros2_workspace/src
cd ~/ros_workspace
colcon build --symlink-install
```

Above commands should execute without any warnings or errors.

Clone `ros2` branch of this repository to your workspace :

```
cd ~/ros_workspace/src
git clone --single-branch --branch ros2 https://github.com/husarion/rosbot_description.git
```

Install depencencies:

```
cd ~/ros_workspace
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
cd ~/ros_workspace
colcon build --symlink-install
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```
source ~/ros_workspace/install/setup.sh
```

## How to use ##

### Map buiding and navigztion example ###

This example allows to build a map and navigate to user defined destinations.

To run the simulation:

```
ros2 launch rosbot_description rosbot_sim.launch.py
```