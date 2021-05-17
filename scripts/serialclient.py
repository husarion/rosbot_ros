#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import array
import errno
import imp
import io
import multiprocessing
import queue
import socket
import struct
import sys
import threading
import time

from serial import Serial, SerialException, SerialTimeoutException

import rclpy
from rclpy.node import Node
from rosserial.std_msgs.msg import Time
from rosserial.rosserial_msgs.msg import TopicInfo
from rosserial.rosserial_msgs.msg import Log
from rosserial.rosserial_msgs.srv import RequestParamRequest, RequestParamResponse

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"

def load_pkg_module(package, directory):
    #check if its in the python path
    path = sys.path
    package_to_import = 'rosserial.' + package + '.' + directory
    try:
        imp.find_module('rosserial')
    except ImportError:
        print("ImportError")
    try:
        _temp = __import__('rosserial')
        m = getattr(_temp, package)
    except ImportError:
        print("ImportError: Cannot import package :" + package_to_import)
        print("sys.path was " + str(path) )
        return None
    return m

def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

def load_service(package,service):
    s = load_pkg_module(package, 'srv')
    s = getattr(s, 'srv')
    srv = getattr(s, service)
    mreq = getattr(s, service+"Request")
    mres = getattr(s, service+"Response")
    return srv,mreq,mres

class Publisher:
    """
        Publisher forwards messages from the serial device to ROS.
    """
    def __init__(self, topic_info, parent):
        self.parent = parent
        """ Create a new publisher. """
        self.topic = topic_info.topic_name
        # find message type
        package, message = topic_info.message_type.split('/')
        self.rosserial_message = load_message(package, message)
        if self.rosserial_message._md5sum == topic_info.md5sum:
            if not package == 'rosbot_ekf':
                message_pkg = __import__(str(package) + '.msg', globals(), locals(), [message], 0)
                self.MessageClass = getattr(message_pkg, message)
                self.publisher = self.parent.create_publisher(self.MessageClass, self.topic, 1)            
            else:
                self.parent.get_logger().info("TODO: Skipped publisher for rosbot_ekf")
                self.parent.get_logger().info(str(topic_info))
        else:
            raise Exception('Checksum does not match: ' + self.rosserial_message._md5sum + ',' + topic_info.md5sum)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        m_rosserial = self.rosserial_message()
        m_rosserial.deserialize(data)
        message_class = str(type(m_rosserial))
        if not message_class == "<class 'rosserial.rosbot_ekf.msg._Imu.Imu'>":
            if message_class == "<class 'rosserial.sensor_msgs.msg._BatteryState.BatteryState'>":
                m_ros2 = self.MessageClass()
                m_ros2.header.frame_id = m_rosserial.header.frame_id
                m_ros2.header.stamp.sec = m_rosserial.header.stamp.secs
                m_ros2.header.stamp.nanosec = m_rosserial.header.stamp.nsecs
                m_ros2.voltage = m_rosserial.voltage
                m_ros2.temperature = float('nan')
                m_ros2.current = m_rosserial.current
                m_ros2.charge = m_rosserial.charge
                m_ros2.capacity = m_rosserial.capacity
                m_ros2.design_capacity = m_rosserial.design_capacity
                m_ros2.percentage = m_rosserial.percentage
                m_ros2.power_supply_status = m_rosserial.power_supply_status
                m_ros2.power_supply_health = m_rosserial.power_supply_health
                m_ros2.power_supply_technology = m_rosserial.power_supply_technology
                m_ros2.present = m_rosserial.present
                m_ros2.cell_voltage = m_rosserial.cell_voltage
                m_ros2.cell_temperature = []
                m_ros2.location = m_rosserial.location
                m_ros2.serial_number = m_rosserial.serial_number
                self.publisher.publish(m_ros2)
            elif message_class == "<class 'rosserial.geometry_msgs.msg._PoseStamped.PoseStamped'>":
                m_ros2 = self.MessageClass()
                m_ros2.header.frame_id = m_rosserial.header.frame_id
                m_ros2.header.stamp.sec = m_rosserial.header.stamp.secs
                m_ros2.header.stamp.nanosec = m_rosserial.header.stamp.nsecs
                m_ros2.pose.position.x = m_rosserial.pose.position.x
                m_ros2.pose.position.y = m_rosserial.pose.position.y
                m_ros2.pose.position.z = m_rosserial.pose.position.z
                m_ros2.pose.orientation.x = m_rosserial.pose.orientation.x
                m_ros2.pose.orientation.y = m_rosserial.pose.orientation.y
                m_ros2.pose.orientation.z = m_rosserial.pose.orientation.z
                m_ros2.pose.orientation.w = m_rosserial.pose.orientation.w
                self.publisher.publish(m_ros2)
            elif message_class == "<class 'rosserial.geometry_msgs.msg._Twist.Twist'>":
                m_ros2 = self.MessageClass()
                m_ros2.linear.x = m_rosserial.linear.x
                m_ros2.linear.y = m_rosserial.linear.y
                m_ros2.linear.z = m_rosserial.linear.z
                m_ros2.angular.x = m_rosserial.angular.x
                m_ros2.angular.y = m_rosserial.angular.y
                m_ros2.angular.z = m_rosserial.angular.z
                self.publisher.publish(m_ros2)
            elif message_class == "<class 'rosserial.sensor_msgs.msg._Imu.Imu'>":
                m_ros2 = self.MessageClass()
                m_ros2.header.frame_id = m_rosserial.header.frame_id
                m_ros2.header.stamp.sec = m_rosserial.header.stamp.secs
                m_ros2.header.stamp.nanosec = m_rosserial.header.stamp.nsecs

                m_ros2.orientation.x = m_rosserial.orientation.x
                m_ros2.orientation.y = m_rosserial.orientation.y
                m_ros2.orientation.z = m_rosserial.orientation.z
                m_ros2.orientation.w = m_rosserial.orientation.w
                m_ros2.orientation_covariance = m_rosserial.orientation_covariance

                m_ros2.angular_velocity.x = m_rosserial.angular_velocity.x
                m_ros2.angular_velocity.y = m_rosserial.angular_velocity.y
                m_ros2.angular_velocity.z = m_rosserial.angular_velocity.z
                m_ros2.angular_velocity_covariance = m_rosserial.angular_velocity_covariance

                m_ros2.linear_acceleration.x = m_rosserial.linear_acceleration.x
                m_ros2.linear_acceleration.y = m_rosserial.linear_acceleration.y
                m_ros2.linear_acceleration.z = m_rosserial.linear_acceleration.z
                m_ros2.linear_acceleration_covariance = m_rosserial.linear_acceleration_covariance

                self.publisher.publish(m_ros2)
            elif message_class == "<class 'rosserial.sensor_msgs.msg._Range.Range'>":
                m_ros2 = self.MessageClass()
                m_ros2.radiation_type = m_rosserial.radiation_type
                m_ros2.field_of_view = m_rosserial.field_of_view
                m_ros2.min_range = m_rosserial.min_range
                m_ros2.max_range = m_rosserial.max_range
                m_ros2.range = m_rosserial.range
                self.publisher.publish(m_ros2)
            elif message_class == "<class 'rosserial.std_msgs.msg._UInt8.UInt8'>":
                m_ros2 = self.MessageClass()
                m_ros2.data = m_rosserial.data
                self.publisher.publish(m_ros2)
            else:
                self.parent.get_logger().info("Unsupported message type: " + message_class)

class Subscriber:
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent
        # find message type
        package, message = topic_info.message_type.split('/')
        self.rosserial_message = load_message(package, message)
        if self.rosserial_message._md5sum == topic_info.md5sum:
            print("TODO: Init subscriber on topic: " + self.topic)
            message_pkg = __import__(str(package) + '.msg', globals(), locals(), [message], 0)
            self.MessageClass = getattr(message_pkg, message)
            self.subscriber = self.parent.create_subscription(self.MessageClass, self.topic, self.callback, 1)
        else:
            raise Exception('Checksum does not match: ' + self.rosserial_message._md5sum + ',' + topic_info.md5sum)

    def callback(self, msg_ros2):
        """ Forward message to serial device. """
        message_class = str(type(msg_ros2))
        if message_class == "<class 'geometry_msgs.msg._twist.Twist'>":
            data_buffer = io.BytesIO()
            msg_rosserial = self.rosserial_message()
            msg_rosserial.linear.x = msg_ros2.linear.x
            msg_rosserial.linear.y = msg_ros2.linear.y
            msg_rosserial.linear.z = msg_ros2.linear.z
            msg_rosserial.angular.x = msg_ros2.angular.x
            msg_rosserial.angular.y = msg_ros2.angular.y
            msg_rosserial.angular.z = msg_ros2.angular.z
            msg_rosserial.serialize(data_buffer)
            self.parent.send(self.id, data_buffer.getvalue())
        elif message_class == "<class 'std_msgs.msg._uint32.UInt32'>":
            data_buffer = io.BytesIO()
            msg_rosserial = self.rosserial_message()
            msg_rosserial.serialize(data_buffer)
            self.parent.send(self.id, data_buffer.getvalue())
        else:
            self.parent.get_logger().info("Unsupported message type: " + message_class)

    def unregister(self):
        self.get_logger().info("Removing subscriber: %s", self.topic)
        self.subscriber.unregister()

class ServiceServer:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent
        self.parent.get_logger().info("Init service server for " + topic_info.message_type)
        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.req_rosserial = getattr(s, service+"Request")
        self.res_rosserial = getattr(s, service+"Response") 
        # srv_pkg = __import__(str(package) + '.srv', globals(), locals(), [service], 0)
        # self.ServiceClass = getattr(srv_pkg, service)
        # self.service_server = self.parent.create_service(self.ServiceClass, self.topic, self.callback)

        # response message
        self.data = None

    def unregister(self):
        self.get_logger().info("Removing service: %s", self.topic)
        self.service.shutdown()

    def callback(self, req_ros2):
        """ Forward request to serial device. """
        data_buffer = io.BytesIO()
        # TODO copy data from req_ro2 to req_rosserial
        req_rosserial = self.req_rosserial()
        req_rosserial.serialize(data_buffer)
        self.response = None
        self.parent.send(self.id, data_buffer.getvalue())
        while self.response is None:
            pass
        return self.response

    def handlePacket(self, data):
        """ Forward response to ROS network. """
        r = self.res_rosserial()
        r.deserialize(data)
        # TODO copy data from r(rosserial) to res(ros2)
        # res = self.ServiceClass.Request()
        # self.response = res


class ServiceClient:
    # TODO Service client is not used in current firmware
    # It may be needed to implement this functionality in future
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.mreq = getattr(s, service+"Request")
        self.mres = getattr(s, service+"Response")
        srv = getattr(s, service)
        self.get_logger().info("Starting service client, waiting for service '" + self.topic + "'")
        # rospy.wait_for_service(self.topic)
        # self.proxy = rospy.ServiceProxy(self.topic, srv)

    def handlePacket(self, data):
        """ Forward request to ROS network. """
        req = self.mreq()
        req.deserialize(data)
        # call service proxy
        resp = self.proxy(req)
        # serialize and publish
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

class SerialClient(Node):
    """
        ServiceServer responds to requests from the serial device.
    """
    header = b'\xff'

    # hydro introduces protocol ver2 which must match node_handle.h
    # The protocol version is sent as the 2nd sync byte emitted by each end
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2

    def __init__(self, timeout=5.0, fix_pyserial_for_test=False):
        super().__init__('serial_node')
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 500000)
        self.port = self.get_parameter('port')
        self.baud = self.get_parameter('baud')

        self.read_lock = threading.RLock()

        self.write_lock = threading.RLock()
        self.write_queue = queue.Queue()
        self.write_thread = None

        self.lastsync = self.get_clock().now()
        self.lastsync_lost = self.get_clock().now()
        self.lastsync_success = self.get_clock().now()
        self.last_read = self.get_clock().now()
        self.last_write = self.get_clock().now()
        self.timeout = timeout
        self.synced = False
        self.fix_pyserial_for_test = fix_pyserial_for_test

        
        self.serial_publishers = dict()  # id:Publishers
        self.serial_subscribers = dict() # topic:Subscriber
        self.serial_services = dict()    # topic:Service

        if self.port is None:
            self.get_logger().error("no port specified, listen for any new port?")
            pass
        else:
            self.get_logger().info("open port " + self.port.value)
            if rclpy.ok():
                try:
                    if self.fix_pyserial_for_test:
                        # see https://github.com/pyserial/pyserial/issues/59
                        self.port = Serial(self.port.value, self.baud.value, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        self.port = Serial(self.port.value, self.baud.value, timeout=self.timeout, write_timeout=10)
                except SerialException as e:
                    self.get_logger().error("Error opening serial")
                    return

        if not rclpy.ok():
            return

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        # rospy.sleep(2.0)
        self.requestTopics()
        self.lastsync = self.get_clock().now()

        read_timer_period = 0.01  # seconds
        self.read_timer = self.create_timer(read_timer_period, self.read_timer_callback)
        write_timer_period = 0.01  # seconds
        self.write_timer = self.create_timer(write_timer_period, self.processWriteQueue)
        

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.get_logger().info('Requesting topics...')

        # TODO remove if possible
        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        self.get_logger().info("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = self.get_clock().now()
                    result.extend(received)
                    bytes_remaining -= len(received)
                else:
                    time.sleep(0.01)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def read_timer_callback(self):
        """ Forward recieved messages to appropriate publisher. """

        # Handle reading.
        data = ''
        read_step = None
        if (self.get_clock().now() - self.lastsync).nanoseconds*1e-9 > (self.timeout * 3):
            if self.synced:
                self.get_logger().error("Lost sync with device, restarting...")
            else:
                self.get_logger().error("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
            self.lastsync_lost = self.get_clock().now()
            self.requestTopics()
            self.lastsync = self.get_clock().now()
        # This try-block is here because we make multiple calls to read(). Any one of them can throw
        # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
        # bottom attempts to reconfigure the topics.
        try:
            with self.read_lock:
                if self.port.inWaiting() < 1:
                    return
            # Find sync flag.
            flag = [0, 0]
            read_step = 'syncflag'
            flag[0] = self.tryRead(1)
            if (flag[0] != self.header):
                return
            # Find protocol version.
            read_step = 'protocol'
            flag[1] = self.tryRead(1)
            if flag[1] != self.protocol_ver:
                self.get_logger().error("Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(flag[1]))
                protocol_ver_msgs = {
                        self.protocol_ver1: 'Rev 0 (rosserial 0.4 and earlier)',
                        self.protocol_ver2: 'Rev 1 (rosserial 0.5+)',
                        b'\xfd': 'Some future rosserial version'
                }
                if flag[1] in protocol_ver_msgs:
                    found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                else:
                    found_ver_msg = "Protocol version of client is unrecognized"
                self.get_logger().info("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                return
            # Read message length, checksum (3 bytes)
            read_step = 'message length'
            msg_len_bytes = self.tryRead(3)
            msg_length, _ = struct.unpack("<hB", msg_len_bytes)
            # Validate message length checksum.
            if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                self.get_logger().info("Wrong checksum for msg length, length %d, dropping message." % (msg_length))
                return
            # Read topic id (2 bytes)
            read_step = 'topic id'
            topic_id_header = self.tryRead(2)
            topic_id, = struct.unpack("<H", topic_id_header)
            # Read serialized message data.
            read_step = 'data'
            try:
                msg = self.tryRead(msg_length)
            except IOError:
                self.get_logger().info("Packet Failed :  Failed to read msg data")
                self.get_logger().info("expected msg length is %d", msg_length)
                raise
            # Reada checksum for topic id and msg
            read_step = 'data checksum'
            chk = self.tryRead(1)
            checksum = sum(array.array('B', topic_id_header + msg + chk))
            # Validate checksum.
            if checksum % 256 == 255:
                self.synced = True
                self.lastsync_success = self.get_clock().now()
                try:
                    self.callbacks[topic_id](msg)
                except KeyError:
                    self.get_logger().error("Tried to publish before configured, topic id %d" % topic_id)
                    self.requestTopics()
                time.sleep(0.001)
            else:
                self.get_logger().info("wrong checksum for topic id and msg")
        except IOError as exc:
            rclpy.get_logger().warn('Last read step: %s' % read_step)
            rclpy.get_logger().warn('Run loop error: %s' % exc)
            # One of the read calls had an issue. Just to be safe, request that the client
            # reinitialize their topics.
            with self.read_lock:
                self.port.flushInput()
            with self.write_lock:
                self.port.flushOutput()
            self.requestTopics()
        # self.txStopRequest()

    def setPublishSize(self, size):
        if self.buffer_out < 0:
            self.buffer_out = size
            self.get_logger().info("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            self.get_logger().info("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def setupPublisher(self, data):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg, self)
            self.get_logger().info("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type) )
            self.serial_publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
        except Exception as e:
            self.get_logger().error("Creation of publisher failed: {0}".format(e))  

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.serial_subscribers.keys()):
                sub = Subscriber(msg, self)
                self.serial_subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                self.get_logger().info("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
            elif msg.message_type != self.serial_subscribers[msg.topic_name].rosserial_message._type:
                old_message_type = self.serial_subscribers[msg.topic_name].rosserial_message._type
                self.serial_subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.serial_subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                self.get_logger().info("Change the message type of subscriber on %s from [%s] to [%s]" % (msg.topic_name, old_message_type, msg.message_type) )
            else:
                self.get_logger().info("Subscriber inicialization fine")

        except Exception as e:
            self.get_logger().error("Creation of subscriber failed: {0}".format(e))

    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.serial_services[msg.topic_name]
                # self.get_logger().error(srv.mreq)
            except KeyError:
                srv = ServiceServer(msg, self)
                self.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.serial_services[msg.topic_name] = srv

            if srv.res_rosserial._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' +
                                srv.res_rosserial._md5sum + ',' + msg.md5sum)
        except Exception as e:
            self.get_logger().error("Creation of service server failed: {0}".format(e))

    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.serial_services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                self.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.serial_services[msg.topic_name] = srv
            if srv.req_rosserial._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            self.get_logger().error("Creation of service server failed: {0}".format(e))

    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.serial_services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                self.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.serial_services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            self.get_logger().error("Creation of service client failed: {0}".format(e))

    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.serial_services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                self.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.serial_services[msg.topic_name] = srv
            if srv.res_rosserial._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.res_rosserial._md5sum + ',' + msg.md5sum)
        except Exception as e:
            self.get_logger().error("Creation of service client failed: {0}".format(e))

    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        now = self.get_clock().now()
        t = Time()
        t.data.nsecs = int(now.nanoseconds % 1e9)
        t.data.secs = int((now.nanoseconds - t.data.nsecs) / 1e9)
        data_buffer = io.BytesIO()
        t.serialize(data_buffer)
        self.send( TopicInfo.ID_TIME, data_buffer.getvalue() )
        self.lastsync = self.get_clock().now()

    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        try:
            print("Get param")
            # param = rospy.get_param(req.name)
        except KeyError:
            self.get_logger().error("Parameter %s does not exist"%req.name)
            return

        if param is None:
            self.get_logger().error("Parameter %s does not exist"%req.name)
            return

        if isinstance(param, dict):
            self.get_logger().error("Cannot send param %s because it is a dictionary"%req.name)
            return
        if not isinstance(param, list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                self.get_logger().error('All Paramers in the list %s must be of the same type'%req.name)
                return
        if t == int or t == bool:
            resp.ints = param
        if t == float:
            resp.floats =param
        if t == str:
            resp.strings = param
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if msg.level == Log.ROSDEBUG:
            self.get_logger().debug(msg.msg)
        elif msg.level == Log.INFO:
            self.get_logger().info(msg.msg)
        elif msg.level == Log.WARN:
            rclpy.get_logger().warn(msg.msg)
        elif msg.level == Log.ERROR:
            self.get_logger().error(msg.msg)
        elif msg.level == Log.FATAL:
            self.get_logger().fatal(msg.msg)

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = self.get_clock().now()

    def _send(self, topic, msg_bytes):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            self.get_logger().error("Message from ROS network dropped: message larger than buffer.\n%s" % msg)
            return -1
        else:
            # frame : header (1b) + version (1b) + msg_len(2b) + msg_len_chk(1b) + topic_id(2b) + msg(nb) + msg_topic_id_chk(1b)
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        if self.write_queue.empty():
            return
        else:
            data = self.write_queue.get()
            try:
                if isinstance(data, tuple):
                    topic, msg = data
                    self._send(topic, msg)
                elif isinstance(data, bytes):
                    self._write(data)
                else:
                    self.get_logger().error("Trying to write invalid data type: %s" % type(data))
            except SerialTimeoutException as exc:
                self.get_logger().error('Write timeout: %s' % exc)
            except RuntimeError as exc:
                self.get_logger().error('Write thread exception: %s' % exc)
