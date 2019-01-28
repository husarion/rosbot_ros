#!/usr/bin/env python
# script based on rosserial with some classes modified
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

import rospy
from rosserial_python import Subscriber, SerialClient, RosSerialServer, load_message
from serial import SerialException, Serial
from time import sleep
import multiprocessing
# from serial_bridge import HusarionSerialClient

import sys

from rosserial_msgs.msg import TopicInfo, Log
from rosserial_msgs.srv import RequestParamRequest, RequestParamResponse

class hSubscriber(Subscriber, object):
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._md5sum == topic_info.md5sum:
            queue_s=1
            buff_s=65536
            self.subscriber = rospy.Subscriber(self.topic, self.message, self.callback, callback_args=None, queue_size=queue_s, buff_size=buff_s, tcp_nodelay=False)
            rospy.loginfo('Init hSubscriber with queue_size: ' + str(queue_s) + ', buff_size: ' + str(buff_s))
        else:
            raise Exception('Checksum does not match: ' + self.message._md5sum + ',' + topic_info.md5sum)


class HusarionSerialClient(SerialClient, object):

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in self.subscribers.keys():
                sub = hSubscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rospy.loginfo("Setup hSubscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = hSubscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rospy.loginfo("Change the message type of subscriber on %s from [%s] to [%s]" % (msg.topic_name, old_message_type, msg.message_type) )
        except Exception as e:
            rospy.logerr("Creation of subscriber failed: %s", e)
    pass

if __name__=="__main__":

    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','500000'))

    # TODO: should these really be global?
    tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
    fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        tcp_portnum = int(sys.argv[2])

    if port_name == "tcp" :
        server = RosSerialServer(tcp_portnum, fork_server)
        rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
        try:
            server.listen()
        except KeyboardInterrupt:
            rospy.loginfo("got keyboard interrupt")
        finally:
            rospy.loginfo("Shutting down")
            for process in multiprocessing.active_children():
                rospy.loginfo("Shutting down process %r", process)
                process.terminate()
                process.join()
            rospy.loginfo("All done")

    else :          # Use serial port
        while not rospy.is_shutdown():
            rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
            try:
                if port_name == '/dev/ttyCORE2':
                    port = Serial(port_name, baud, timeout=0.25, rtscts=True, dsrdtr=True, )
                else:
                    port = Serial(port_name, baud, timeout=0.25)
                client = HusarionSerialClient(port, 0)
                client.run()
            except KeyboardInterrupt:
                break
            except SerialException:
                sleep(1.0)
                continue
            except OSError:
                sleep(1.0)
                continue
