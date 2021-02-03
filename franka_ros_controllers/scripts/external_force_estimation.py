#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb

import ros_helper
import franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import PointStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt


def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_base_frame
    end_effector_wrench_in_base_frame = data

def get_xy_wrench_world(wrench_list):
    return [wrench_list[0], wrench_list[2], wrench_list[-2]]
        
def pivot_xyz_callback(data):
    global pivot_xyz
    pivot_xyz = data


if __name__ == '__main__':

    rospy.init_node("external_force_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    # initialize globals
    pivot_xyz, end_effector_wrench_in_base_frame = None, None

    # setting up subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_xyz", PointStamped, pivot_xyz_callback)
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)

    # make sure subscribers are receiving commands
    print("Waiting for pivot estimate to stabilize")
    while pivot_xyz is None:
        rospy.sleep(0.1)

    print("Waiting for wrench estimate")
    while end_effector_wrench_in_base_frame is None:
        rospy.sleep(0.1)

    print("Starting to publish external force estimates")
    while not rospy.is_shutdown():

        # face_center franka pose
        endpoint_pose_franka = arm.endpoint_pose()

        # face_center list
        endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

        # get vector from pivot to hand
        pivot_xyz_array = np.array(ros_helper.point_stamped2list(pivot_xyz))
        franka_xyz_array = endpoint_pose_franka['position']
        pivot_to_hand = franka_xyz_array - pivot_xyz_array

        # end effector wrench about pivot
        pivot_wrench_base_frame = ros_helper.wrench_reference_point_change(
            end_effector_wrench_in_base_frame, pivot_to_hand)

        # pivot wrench 2D
        pivot_wrench_2D_base = get_xy_wrench_world(ros_helper.wrench_stamped2list
            (pivot_wrench_base_frame))
        print(pivot_wrench_2D_base)

        # # update messages
        # position_msg.data = ee_pos_contact_frame
        # velocity_msg.data = ee_vel_contact_frame

        # # publish
        # generalized_positions_pub.publish(position_msg)
        # generalized_velocities_pub.publish(velocity_msg)
        rate.sleep()    