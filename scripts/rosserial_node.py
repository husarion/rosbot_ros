#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serialclient import SerialClient

def main(args=None):
    rclpy.init(args=args)
    sc = SerialClient()
    rclpy.spin(sc)
    sc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()