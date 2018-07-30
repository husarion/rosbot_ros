# rosbot_description #

URDF model for Gazebo integrated with ROS.

## 1. Using the package. ##

Description of the available .sh file:

run_gazebo.sh - 
Killing all gzserver, gzclient, rviz, roscore and rosmaster processes.

Runnig gazebo with ROSbot model and operating environment are defined in /rosbot_gazebo/launch/rosbot_world.launch.

run_rviz.sh - 
Runnig launch file runnig Rviz with implemented parameters.

run_nav.sh - 
Runnig launch file with already created map and "move base" node.

run_teleop.sh -
Runnig teleop_twist_keyboard to allow control of the robot.

## 2. Tips ##

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

## 3. Tutorials ##

### Creating, saving and loading the Map ###

Run the following commands below. Use the teleop to move the robot around to create an accurate and thorough map.

In Terminal 1, launch the Gazebo world
```
roslaunch rosbot_gazebo rosbot_world.launch
```
In Terminal 2, start map building
```
roslaunch rosbot_navigation gmapping_demo.launch
```

In Terminal 3, launch rviz and set the following parameters:
```
roslaunch rosbot_description rosbot_rviz_gmapping.launch
```

In Terminal 4, start teleop and create a map
```
roslaunch rosbot_navigation rosbot_teleop.launch
```
In Terminal 5, save the map to some file path 
```
rosrun map_server map_saver -f ~/ros_workspace/src/rosbot_description/src/rosbot_navigation/maps/test_map
```

Now to make saved map loading possible you have to close all previous terminals and run the following commands below. Once loaded, use rviz to set 2D Nav Goal and the robot will autonomously reache the indicated position
In Terminal 1, launch the Gazebo world
```
roslaunch rosbot_gazebo rosbot_world.launch
```

In Terminal 2, start map building
```
roslaunch rosbot_navigation amcl_demo.launch
```

In Terminal 3, launch rviz
```
roslaunch rosbot_description rosbot_rviz_amcl.launch
```	



