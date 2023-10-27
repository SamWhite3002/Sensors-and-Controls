#!/usr/bin/python

import actionlib
import rospy
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

# Define the action server for the gripper
ACTION_SERVER = 'head_controller/point_head'

class robot_head:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(ACTION_SERVER, PointHeadAction)
        rospy.loginfo("Waiting for robot_head...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        """
        Point the robot's head to a specific location in space.

        Args:
            x (float): X-coordinate of the target point.
            y (float): Y-coordinate of the target point.
            z (float): Z-coordinate of the target point.
            frame (str): The reference frame in which the target point is expressed.
            duration (float): Minimum duration to reach the target (default is 1.0 second).
        """
        goal = PointHeadGoal()
        point = PointStamped()
        point.header.frame_id = frame
        point.point.x = x
        point.point.y = y
        point.point.z = z
        goal.target = point
        goal.pointing_frame = "base_link"
        goal.pointing_axis.x = 1
        goal.pointing_axis.y = 0
        goal.pointing_axis.z = 0
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 1.0

        self.client.send_goal_and_wait(goal, rospy.Duration(3))
        self.result = self.client.get_result()
