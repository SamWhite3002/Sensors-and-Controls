#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class robot_interface:
    def __init__(self):
        #Initisliaze ROS node 
        rospy.init_node("fetch_builder", anonymous=True)    
        #Initislize end effector variables
        self.x = None
        self.y = None
        self.z = None
        self.a = None
        self.b = None
        self.c = None
        self.d = None
        self.callback = 0
        self.status = 0
        #Create publisher; Status
        self.pub = rospy.Publisher('Status', Bool, queue_size=15)
        #Create rate object; 15Hz = 15 messages are published per second
        self.rate = rospy.Rate(15)  

    def callback(self, data): #Process messages from Pose rostopic
        #make status = 0
        self.get_status(0)
        #publish status
        self.publish_status()
        #get info from pose and assign to variables
        pose = data.pose
        self.x, self.y, self.z = pose.position.x, pose.position.y, pose.position.z
        self.a, self.b, self.c, self.d = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        #
        self.callback = 1
        rospy.loginfo(rospy.get_caller_id() + "%.2f %.2f %.2f %.2f %.2f %.2f %.2f", self.x, self.y, self.z, self.a, self.b, self.c, self.d)

    def subscribe(self): #subscribe to the Pose rostopic
        rospy.Subscriber("Pose", PoseStamped, self.callback)

    def get_status(self, status):
        self.status = status

    def publish_status(self):
        self.pub.publish(self.status)
        self.rate.sleep()

    
