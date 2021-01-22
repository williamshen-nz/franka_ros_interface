#!/usr/bin/env python

import rospy
import tf
import pdb

from geometry_msgs.msg import PoseStamped
from franka_interface import ArmInterface 
from ros_helper import list2pose_stamped

def franka_pose2list(arm_pose):
    position_list = arm_pose['position'].tolist()
    orientation_list = [arm_pose['orientation'].x,
                               arm_pose['orientation'].y,
                               arm_pose['orientation'].z,
                               arm_pose['orientation'].w]
    return position_list + orientation_list


if __name__ == '__main__':

    #1. initialize node
    rospy.init_node('ee_pose_in_world_from_franka',
        anonymous=True)
    rate = rospy.Rate(30.)             

    #2. initialize arm
    arm = ArmInterface()

    #3. Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    #4. Define rostopic publishers
    ee_pose_in_world_from_franka_pub = rospy.Publisher('/ee_pose_in_world_from_franka_publisher', 
        PoseStamped, queue_size = 10)

    #5. Run node at rate
    while not rospy.is_shutdown():

        #5a. get ee pose
        ee_pose_in_world_franka = arm.endpoint_pose()

        #5b. convert ee pose to list
        ee_pose_in_world_list = franka_pose2list(ee_pose_in_world_franka)

        #5c. convert ee pose list to pose stamped
        ee_pose_in_world_pose_stamped = list2pose_stamped(ee_pose_in_world_list)

        #5d. publish and sleep
        ee_pose_in_world_from_franka_pub.publish(ee_pose_in_world_pose_stamped)
        rate.sleep()