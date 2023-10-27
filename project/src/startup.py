#!/usr/bin/env python 

import rospy
import time

from robot_arm import robot_arm
from robot_interface import robot_interface
from robot_gripper import robot_gripper
from robot_head import robot_head
from grip_interface import grip_interface

if __name__ == "__main__":
    # Setup clients
    armRos = robot_interface()
    arm = robot_arm()
    gripperRos = grip_interface()
    gripper = robot_gripper()
    head = robot_head()
    head.look_at(0.7, 0, 0.7, "base_link")
    print("Head Tilted")
    
    while 1:
        try:
            armRos.subscribe()
            gripperRos.subscribe()
            break
        except:
            rospy.loginfo('WAITING')

    
    while not rospy.is_shutdown():
        armRos.publish_status()
        gripperRos.publish_grip_status()
        
        if(gripperRos.grip_callback == 1):
            gripper.grip(gripperRos.grip_state)
            gripperRos.set_grip_status(1)
            gripperRos.grip_callback = 0

        if (armRos.callback == 1):
            arm.MoveToPose(Ros.x, Ros.y, Ros.z, Ros.a, Ros.b, Ros.c, Ros.d)
            armRos.get_status(1)
            armRos.callback = 0
