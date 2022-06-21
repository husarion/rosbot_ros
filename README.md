# rosbot_description #

ROS packages for ROSbot 2.0 and ROSbot 2.0 Pro.


# Quick start (real ROSbot) ## 
## Prerequisites ###

Make sure the right version of firmware for STM32F4 MCU is flashed on ROSbot. To flash the right firmware, open ROSbot's terminal or connect via `ssh` and execute this command:
 - for differential drive (regular wheels):
```bash
docker run --privileged --rm husarion/rosbot:melodic ./flash_firmware.sh diff
```
- for omnidirectional wheeled ROSbot (mecanum wheels):
```bash
docker run --privileged --rm husarion/rosbot:melodic ./flash_firmware.sh mecanum
```
## Control ROSbot from RViz running on your laptop - Simultaneous Localization And Mapping
### In LAN:
Connect your ROSbot and laptop to the same Wi-Fi network, navigate to `demo/` folder and execute:

- On laptop:

    ```bash
    xhost local:root
    docker compose -f compose.pc-gpu.yaml -f compose.pc.lan.yaml up
    ```
    >**Tip:**
    >
    >If your computer is not equipped with GPU you can replace `compose.pc-gpu.yaml` file with `compose.pc-cpu.yaml`

- On ROSbot:

    ```bash
    docker compose -f compose.rosbot.yaml -f compose.rosbot.nav2-slam.yaml -f compose.rosbot.lan.yaml up
    ```

Prepare map with Rviz2 using `2D Goal Pose`.

### Over The Internet (Husarnet)
1. Connect your ROSbot and laptop to the same or different networks
2. Create `.env` file based on the `demo/.env.template` and paste your own Husarnet Join Code here
3. Navigate to `demo/` folder and execute: 

- On laptop:

    ```bash
    xhost local:root
    docker compose -f compose.pc-gpu.yaml -f compose.pc.husarnet.yaml up
    ```

- On ROSbot:

    ```bash
    docker compose -f compose.rosbot.yaml -f compose.rosbot.nav2-slam.yaml -f compose.rosbot.lan.yaml up
    ```

4. Prepare map with Rviz2 using `2D Goal Pose`.


### No network: run everything on  the ROSbot
1. Connect the screen, mouse and keyboard to the rear panel of the ROSbot, and open a terminal app.
2. Navigate to `demo/` folder and execute: 

- On ROSbot:

  ```bash
  xhost local:root
  docker compose -f compose.pc.yaml -f compose.rosbot.yaml up
  ```

3. Prepare map with Rviz2 using `2D Goal Pose`.

## Saving the Map
If the map is ready, open a new terminal and run this commands:

- On ROSbot:

  ```bash
  docker exec -it nav2-slam bash
  source /opt/ros/galactic/setup.bash

  cd /ros2_ws/src/husarion_nav2/config
  ros2 run nav2_map_server map_saver_cli --fmt pgn -f map
  ```
Your map has been saved in docker volume and is now in the `config/` folder.

## Control ROSbot from RViz running on your laptop (amcl localization)
In order for the robot to be able to use the previously prepared map for localization and navigating, launch:

  - On laptop:
      >**Tip:**
      >
      >If your computer is not equipped with GPU you can replace `compose.pc-gpu.yaml` file with `compose.pc-cpu.yaml`
      
      Navigate to `/demo` folder and comment out the line of the `compose.pc-***.yaml` file so that the `rviz` node uses the `rosbot_pro_localization.rviz` configuration file. It should looks like this:

      ```yaml
    # ROS Galactic
    rviz:
        image: husarion/rviz2
        ...

        volumes: 
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            # - ./../config/rosbot_pro_mapping.rviz:/root/.rviz2/default.rviz
            - ./../config/rosbot_pro_localization.rviz:/root/.rviz2/default.rviz
      ```

      Next, run compose files: 

      ```bash
      xhost local:root
      docker compose -f compose.pc-gpu.yaml -f compose.pc.lan.yaml up
      ```
      

  - On ROSbot:

      ```bash
      docker compose -f compose.rosbot.yaml -f compose.rosbot.nav2-amcl.yaml -f compose.rosbot.lan.yaml up
      ```

The above commands run an example on a lan network, but the same works for other types of connection. For example, if you are connected via the internet (Husarnet), replace `compose.rosbot.lan.yaml` file with `compose.rosbot.husarnet.yaml`.

# Quick start (simulation) #

## Installation ##

We assume that you are working on Ubuntu 16.04 and already have installed ROS Kinetic. If not, follow the [ROS install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Prepare the repository:
```bash
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

```bash
cd ~/ros_workspace/src
git clone https://github.com/husarion/rosbot_description.git
```

Install depencencies:

```bash
cd ~/ros_workspace
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/ros_workspace
catkin_make
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```bash
source ~/ros_workspace/devel/setup.sh
```

## Creating, saving and loading the Map with Gazebo ##

Run the following commands below. Use the teleop to move the robot around to create an accurate and thorough map.

In Terminal 1, launch the Gazebo simulation:

```bash
roslaunch rosbot_description rosbot_rviz_gmapping.launch
```

In Terminal 2, start teleop and drive the ROSbot, observe in Rviz as the map is created:

```bash
roslaunch rosbot_navigation rosbot_teleop.launch
```

When you are satisfied with created map, you can save it. Open new terminal and save the map to some given path: 

```bash
rosrun map_server map_saver -f ~/ros_workspace/src/rosbot_description/src/rosbot_navigation/maps/test_map
```

Now to make saved map loading possible you have to close all previous terminals and run the following commands below. Once loaded, use rviz to set 2D Nav Goal and the robot will autonomously reach the indicated position

In Terminal 1, launch the Gazebo simulation

```bash
roslaunch rosbot_description rosbot_rviz_amcl.launch
```

## Creating, saving and loading the Map with Gazebo and Docker ##
The only thing you need is Docker, so make sure you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and [Docker Compose v2](https://docs.docker.com/compose/cli-command/#install-on-linux).

To launch simulations, if you haven't done so already, you have to copy project git repository and run `compose.simulation.gmapping.yaml` located in the `demo` folder. Execute in a termianl on your laptop:

```bash
git clone https://github.com/husarion/rosbot_description.git
cd demo/

xhost local:root
docker compose -f compose.simulation.gmapping.yaml up --build
```

In another termianl, start teleop and drive the ROSbot, observe in Rviz as the map is created:
```bash
docker exec -it rosbot_ros_simulation bash

source /opt/ros/melodic/setup.bash
source devel/setup.sh

roslaunch rosbot_navigation rosbot_teleop.launch
```
When you are satisfied with created map, you can save it. Save the map to some given volume path:
```bash
rosrun map_server map_saver -f /ros_ws/src/rosbot_navigation/maps/test_map
```

Close all previous terminals and run the following commands below. Once loaded, use rviz to set 2D Nav Goal and the robot will autonomously reach the indicated position. Navigate to `demo/` folder and execute:
```bash
xhost local:root
docker compose -f compose.simulation.amcl.yaml up --build
```

>**Tip:**
>
>If you have any problems with laser scan it probably means that you don't have a dedicated graphic card (or lack appropriate drivers). If that's the case then you'll have to change couple of things in `/rosbot_description/urdf/rosbot_gazebo` file: <br><br>
>Find:   `<!-- If you cant't use your GPU comment RpLidar using GPU and uncomment RpLidar using CPU gazebo plugin. -->`
next coment RpLidar using GPU using `<!-- -->` from `<gazebo>` to `</gazebo>` like below:
> ```xml
> <!-- gazebo reference="rplidar">
>   <sensor type="gpu_ray" name="head_rplidar_sensor">
>     <pose>0 0 0 0 0 0</pose>
>     <visualize>false</visualize>
>     <update_rate>40</update_rate>
>     <ray>
>       <scan>
>         <horizontal>
>           <samples>720</samples>
>           <resolution>1</resolution>
>           <min_angle>-3.14159265</min_angle>
>           <max_angle>3.14159265</max_angle>
>         </horizontal>
>       </scan>
>       <range>
>         <min>0.2</min>
>         <max>30.0</max>
>         <resolution>0.01</resolution>
>       </range>
>       <noise>
>         <type>gaussian</type>
>         <mean>0.0</mean>
>         <stddev>0.01</stddev>
>       </noise>
>     </ray>
>     <plugin name="gazebo_ros_head_rplidar_controller" 
>filename="libgazebo_ros_gpu_laser.so">
>      <topicName>/rosbot/laser/scan</topicName>
>       <frameName>rplidar</frameName>
>     </plugin>
>   </sensor>
> </gazebo -->
>```
>
>Now uncomment RpLidar using CPU plugin removing `<!-- -->`.
>
>If you want to make your laser scan visible just change:
>```xml
><visualize>false</visualize>
>```
>to:
>```xml
><visualize>true</visualize>
>```
>in the same plug in.