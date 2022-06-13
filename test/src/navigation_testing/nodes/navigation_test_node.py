#!/usr/bin/env python
import sys,unittest,time

import rospy,rostest

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib

PKG = "navigation_testing"
NAME = "navigation_test"

class NavTest():
    def __init__(self,*args):
        # load parameters
        self.goal_coords = [-1,1]
        self.test_timeout = rospy.Duration(3000)
    # navigation test
    def test_nav(self):
        # start test node
        rospy.init_node(NAME,anonymous=True)
        # create action client
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
         # create goal to be sent
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x  = self.goal_coords[0]
        goal.target_pose.pose.position.y  = self.goal_coords[1]
        goal.target_pose.pose.orientation.w  = 1.0
        client.send_goal(goal)
        # wait for goal result
        result = client.wait_for_result(self.test_timeout)
        if result:
            if client.get_state() == 3:
                print("success")
        else:
            # If navigation takes too long, cancel all goals
            print(result)
if __name__ == "__main__":
    test = NavTest()
    test.test_nav()