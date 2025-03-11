# ROSbot XL Manipulation

## Gamepad

After running the ROSbot XL Manipulation Package, you should be able to control the manipulator. The easiest way to move the manipulator is to connect a gamepad and steer the robot. The graphic below shows how to steer the manipulator using a gamepad.

![gamepad_manipulation](.docs/gamepad_manipulation.drawio.png)

## RViz Moveit

To move the manipulator using RViz, you need to build the code first. Then run:

```bash
ros2 launch open_manipulator_x_moveit rviz.launch.py
```

After that, RViz with the Moveit configuration will appear.

![moveit_manipulation](.docs/moveit_manipulation.png)
