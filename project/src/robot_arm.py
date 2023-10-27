#!/usr/bin/python

import copy
import actionlib
import rospy

from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped

class robot_arm:
    def __init__(self):
        self.moveGroup = MoveGroupInterface("arm_with_torso", "base_link")
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
  
    def move_to_pose(self, X, Y, Z, x, y, z, w):
        pose_stamped = self.create_pose_stamped(X, Y, Z, x, y, z, w)
        self.moveGroup.moveToPose(pose_stamped, 'gripper_link')
        self.result = self.moveGroup.get_move_action().get_result()

    def create_pose_stamped(self, X, Y, Z, x, y, z, w):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = X
        pose_stamped.pose.position.y = Y
        pose_stamped.pose.position.z = Z
        pose_stamped.pose.orientation.x = x
        pose_stamped.pose.orientation.y = y
        pose_stamped.pose.orientation.z = z
        pose_stamped.pose.orientation.w = w
        return pose_stamped

if __name__ == "__main__":
    try:
        rospy.init_node("robot_arm_node")
        arm = RobotArm()
        arm.move_to_pose(X, Y, Z, x, y, z, w)
    except rospy.ROSInterruptException:
        pass

