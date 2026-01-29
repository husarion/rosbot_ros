#!/usr/bin/env python3
import argparse
import os
import serial
import sys
import time

from rosbot_ros.rosbot_utils.rosbot_utils.mcu_manager_ftdi import McuManagerFTDI
from rosbot_ros.rosbot_utils.rosbot_utils.mcu_manager_uart import McuManagerUART
from rosbot_utils.utils import find_device_port


def configure_robot(port: str, namespace: str, baudrate: int, timeout: float = 5.0) -> bool:
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)
        
        # Clear any pending data
        ser.reset_input_buffer()
        
        print("Waiting for communication...")
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line != "READY":
            print(f"✗ Unexpected response: '{line}'")
            ser.close()
            return False
        
        print(f"Sending namespace: {namespace}")
        ser.write(f"NS:{namespace}\n".encode('utf-8'))
        ser.flush()
        
        # Wait for ACK
        ack = ser.readline().decode('utf-8', errors='ignore').strip()
        ser.close()
        
        if ack == "ACK":
            print(f"✓ Robot configured with namespace: /{namespace}")
            return True
        else:
            print(f"✗ No ACK received: '{ack}'")
            return False
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except KeyboardInterrupt:
        print("\nAborted.")
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description="Configure robot namespace via serial pre-communication.")
    parser.add_argument(
        "--robot-model",
        required=True,
        default=os.getenv("ROBOT_MODEL_NAME"),
        choices=["rosbot", "rosbot_xl"],
        help="Specify the robot model",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Flash via USB. Automatically set for ROSbot XL; other ROSbots use UART by default. (You can flash firmware to ROSbot via USB-A from your PC)",
    )
    pcb_serial_port = find_device_port("0403", "6015", "/dev/ttyUSB0")
    parser.add_argument(
        "-p",
        "--port",
        default=pcb_serial_port,
        help="Specify the communication port",
    )
    parser.add_argument(
        "--namespace",
        default="",
        help="Specify the robot namespace",
    )
    parser.add_argument(
        "-b"
        "--baudrate",
        type=int,
        default=921600,
        help="Specify the serial communication baudrate",
    )
    args = parser.parse_args(args)

    robot_model = args.robot_model
    if robot_model == "rosbot_xl":
        args.usb = True


    try:
        if args.usb:
            mcu_manager = McuManagerFTDI()
            mcu_manager.reset_mcu()
        else:
            mcu_manager = McuManagerUART()
            mcu_manager.reset_mcu()
    except Exception as e:
        print(f"ERROR: {e}")

    success = configure_robot(args.port, args.namespace, args.baudrate)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()