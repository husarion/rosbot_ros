# Developer info and tools

## USB-A connection

You can connect with the ROSbot hardware on your own computer. To establish a connection, connect your computer to the robot using a USB-A cable. Then build the code locally and specify via the serial_port argument which processor should be used to establish the connection.

```bash
ros2 launch rosbot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
```

The hardware checks the connection via USB-A only during initialization and when btn1 or btn2 is pressed, so while executing the above command, hold down the reset button together with bnt1/bnt2 and release the reset button. After establishing a connection, you can release bnt1/bnt2.

## pre-commit

[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run tests
pre-commit run -a
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.
