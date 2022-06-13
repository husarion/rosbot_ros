#!/usr/bin/env python
import sys,unittest,time

import rospy,rostest

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib

PKG = "navigation_testing"
NAME = "navigation_test"

class NavTest(unittest.TestCase):
    def __init__(self,*args):
        super(NavTest,self).__init__(*args)
        # load parameters
        self.goal_coords = (rospy.get_param("goal_x"),rospy.get_param("goal_y"))
        self.test_timeout = rospy.Duration(rospy.get_param("test_timeout"))
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
            # Check goal state, 3 means goal succeeded: http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html
            if client.get_state() == 3:
                self.assert_(True)
            else:
                self.assert_(False,"Navigation goal aborted")
        else:
            # If navigation takes too long, cancel all goals
            client.cancel_all_goals()
            self.assert_(False,"nvaigation timeout")
            
if __name__ == "__main__":
    rostest.rosrun(PKG,NAME,NavTest,sys.argv)
