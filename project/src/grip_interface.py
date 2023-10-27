#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

class grip_interface:
    def __init__(self):
        # Variables for end effector pose
        self.grip_state = False
        self.grip_callback = 0
        self.grip_status = 0
        
        # Publisher for grip status
        self.grip_pub = rospy.Publisher('Grip_status', Bool, queue_size=15)
        self.rate = rospy.Rate(15)

    def grip_callback(self, data):
        self.set_grip_status(0)
        self.publish_grip_status()
        
        self.grip_state = data.data
        self.grip_callback = 1

        rospy.loginfo(rospy.get_caller_id() + "%r", self.grip_state)

    def subscribe(self):
        rospy.Subscriber("Grip_state", Bool, self.grip_callback)

    def publish_grip_status(self):
        self.grip_pub.publish(self.grip_status)
        self.rate.sleep()

    def set_grip_status(self, grip_status):
        self.grip_status = grip_status


