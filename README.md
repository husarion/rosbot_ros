# rosbot_description #

URDF model for Gazebo integrated with ROS.

## Installation. ## 

We assume that you are working on Ubuntu 16.04 and already have installed ROS Kinetic. If not, follow the [ROS install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Prepare the repository:
```
cd ~
mkdir ros_workspace
mkdir ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
cd ~/ros_workspace
catkin_make
```

Above commands should execute without any warnings or errors.


Clone this repository to your workspace:

```
cd ~/ros_workspace/src
git clone https://github.com/husarion/rosbot_description.git
```

Install depencencies:

```
cd ~/ros_workspace
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
cd ~/ros_workspace
catkin_make
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```
source ~/ros_workspace/devel/setup.sh
```

## How to use ##

### Creating, saving and loading the Map ###

Run the following commands below. Use the teleop to move the robot around to create an accurate and thorough map.

In Terminal 1, launch the Gazebo simulation:

```
roslaunch rosbot_description rosbot_rviz_gmapping.launch
```

In Terminal 2, start teleop and drive the ROSbot, observe in Rviz as the map is created:

```
roslaunch rosbot_navigation rosbot_teleop.launch
```

When you are satisfied with created map, you can save it. Open new terminal and save the map to some given path: 

```
rosrun map_server map_saver -f ~/ros_workspace/src/rosbot_description/src/rosbot_navigation/maps/test_map
```

Now to make saved map loading possible you have to close all previous terminals and run the following commands below. Once loaded, use rviz to set 2D Nav Goal and the robot will autonomously reach the indicated position

In Terminal 1, launch the Gazebo simulation

```
roslaunch rosbot_description rosbot_rviz_amcl.launch
```

## Tips ##

If you have any problems with laser scan it probably means that you don't have a dedicated graphic card (or lack appropriate drivers). If that's the case then you'll have to change couple of things in /rosbot_description/urdf/rosbot_gazebo file:

Find:   `<!-- If you cant't use your GPU comment RpLidar using GPU and uncomment RpLidar using CPU gazebo plugin. -->`
next coment RpLidar using GPU using `<!-- -->` from `<gazebo>` to `</gazebo>` like below:

 ```
 <!-- gazebo reference="rplidar">
   <sensor type="gpu_ray" name="head_rplidar_sensor">
     <pose>0 0 0 0 0 0</pose>
     <visualize>false</visualize>
     <update_rate>40</update_rate>
     <ray>
       <scan>
         <horizontal>
           <samples>720</samples>
           <resolution>1</resolution>
           <min_angle>-3.14159265</min_angle>
           <max_angle>3.14159265</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.2</min>
         <max>30.0</max>
         <resolution>0.01</resolution>
       </range>
       <noise>
         <type>gaussian</type>
         <mean>0.0</mean>
         <stddev>0.01</stddev>
       </noise>
     </ray>
     <plugin name="gazebo_ros_head_rplidar_controller" filename="libgazebo_ros_gpu_laser.so">
       <topicName>/rosbot/laser/scan</topicName>
       <frameName>rplidar</frameName>
     </plugin>
   </sensor>
 </gazebo -->
```

Now uncomment RpLidar using CPU plugin removing `<!-- -->`.

If you want to make your laser scan visible just change:
```
<visualize>false</visualize>
```
to:
```
<visualize>true</visualize>
```
in the same plug in.