#!/usr/bin/python

import rospy
import actionlib
import control_msgs.msg

# Define the action server for the gripper
ACTION_SERVER = 'gripper_controller/gripper_action'

class robot_gripper:
    def __init__(self):
        # Create a simple action client for the gripper action server
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server()

    def grip(self, close):
        goal = control_msgs.msg.GripperCommandGoal()

        if close:
            # When closing, set the maximum effort to 150
            goal.command.max_effort = 150
        else:
            # When opening, set the desired position to 0.10
            goal.command.position = 0.10

        # Send the goal to the gripper action server and wait for the result
        self._client.send_goal(goal)
        self._client.wait_for_result()

        # Store the result for reference
        self.result = self._client.get_result()
