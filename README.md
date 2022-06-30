# rosbot_ros #

ROS packages for ROSbot 2.0 and ROSbot 2.0 Pro.


# Quick start (real ROSbot) ## 
## Prerequisites ###

The only thing you need is Docker, so make sure you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and [Docker Compose v2](https://docs.docker.com/compose/cli-command/#install-on-linux).

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
1. Create `demo/.env` file based on the `demo/.env.template` and complete the necessary information.

2. Connect your ROSbot and laptop to the same Wi-Fi network, navigate to `demo/` folder and execute:

- On laptop:

    ```bash
    xhost local:root
    docker compose -f compose.rviz.yaml -f compose.rviz.lan.yaml up

- On ROSbot:

    ```bash
    docker compose -f compose.rosbot.yaml -f compose.rosbot.slam-toolbox.yaml -f compose.rosbot.lan.yaml up
    ```

Prepare map with Rviz2 using `2D Goal Pose`.

### Over The Internet (Husarnet)
1. Connect your ROSbot and laptop to the same or different networks
2. Create `demo/.env` file based on the `demo/.env.template` and paste your own Husarnet Join Code here
3. Navigate to `demo/` folder and execute: 

- On laptop:

    ```bash
    xhost local:root
    docker compose -f compose.rviz.yaml -f compose.rviz.husarnet.yaml up
    ```

- On ROSbot:

    ```bash
    docker compose -f compose.rosbot.yaml -f compose.rosbot.slam-toolbox.yaml -f compose.rosbot.lan.yaml up
    ```

4. Prepare map with Rviz2 using `2D Goal Pose`.


### No network: run everything on  the ROSbot
1. Connect the screen, mouse and keyboard to the rear panel of the ROSbot, and open a terminal app.
2. Navigate to `demo/` folder and execute: 

- On ROSbot:

  ```bash
  xhost local:root
  docker compose -f compose.rviz.yaml -f compose.rosbot.yaml -f compose.rosbot.slam-toolbox.yaml up
  ```

3. Prepare map with Rviz2 using `2D Goal Pose`.

## Saving the Map
If the map is ready, open a new terminal, navigate to `demo/` folder and execute: 

- On ROSbot:

  ```bash
  ./map-save.sh
  ```
Your map has been saved in docker volume and is now in the `maps/` folder.

## Control ROSbot from RViz running on your laptop (amcl localization)
In order for the robot to be able to use the previously prepared map for localization and navigating, launch:

  - On laptop:
      
      Navigate to `/demo` folder and comment out the line of the `compose.rviz.yaml` file so that the `rviz` node uses the `rosbot_pro_localization.rviz` configuration file. It should looks like this:

      ```yaml
    # ROS Galactic
    rviz:
        image: husarion/rviz2
        ...

        volumes: 
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            # - ./config/rosbot_pro_mapping.rviz:/root/.rviz2/default.rviz
            - ./config/rosbot_pro_localization.rviz:/root/.rviz2/default.rviz
    
      ```

      Next, run compose files: 

      ```bash
      xhost local:root
      docker compose -f compose.rviz.yaml -f compose.rviz.lan.yaml up
      ```
      

  - On ROSbot:

      ```bash
      docker compose -f compose.rosbot.yaml -f compose.rosbot.amcl.yaml -f compose.rosbot.lan.yaml up
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
git clone https://github.com/husarion/rosbot_ros.git
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
rosrun map_server map_saver -f ~/ros_workspace/src/rosbot_ros/src/rosbot_navigation/maps/test_map
```

Now to make saved map loading possible you have to close all previous terminals and run the following commands below. Once loaded, use rviz to set 2D Nav Goal and the robot will autonomously reach the indicated position

In Terminal 1, launch the Gazebo simulation

```bash
roslaunch rosbot_description rosbot_rviz_amcl.launch
```

## Creating, Saving and Loading the Map with Gazebo and Docker
The only thing you need is Docker, so make sure you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and [Docker Compose v2](https://docs.docker.com/compose/cli-command/#install-on-linux).

To launch simulation, if you haven't done so already, you have to copy project git repository and run compose files located in the demo folder. Appropriate compose files configuration will launch Gazebo, rviz or rviz2 node and a package providing navigation and localization algorithms.

### Nav2 + SlamToolbox (Galactic)
To launch `Navigation2` and `SlamToolbox` with Gazebo, execute this lines in a termianl on your laptop:

```bash
git clone https://github.com/husarion/rosbot_ros.git
cd demo/

xhost local:root
docker compose -f compose.simulation.yaml -f compose.rviz.yaml -f compose.rosbot.slam-toolbox.yaml up --build
```

Prepare map with Rviz2 using `2D Goal Pose` and [save the map](#3-saving-the-map).

Next, see what the `compose.rviz.yaml` file should look like ([link](#4-amcl-localization---control-rosbot-from-rviz-running-on-laptop)) and  launch `Navigation2 stack` with `AMLC`:

```bash
xhost local:root
docker compose -f compose.simulation.yaml -f compose.rviz.yaml -f compose.rosbot.amcl.yaml up --build
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