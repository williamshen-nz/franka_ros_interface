#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb

import ros_helper
import franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt


def get_hand_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def update_sliding_velocity(x0, z0, contact_pose, contact_vel):

    contact_pose_stamped = ros_helper.list2pose_stamped(contact_pose)
    contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)
    
    # 2-D unit contact normal in world frame
    e_n = contact_pose_homog[:3, 0]
    n2D = e_n[[0,2]]
    e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

    # 2-D unit contact tangent in world frame
    e_t = contact_pose_homog[:3, 1]
    t2D = e_t[[0,2]]
    e_t2D = t2D/np.sqrt(np.sum(t2D ** 2))

    # 2-D angular velocity
    theta_dot = contact_vel[4]

    # xc and zc
    xc, zc = contact_pose[0] - x0, contact_pose[2] - z0

    # compute velocity jacobian between world frame (x, z, tht) and contact frame (n, t, tht)
    velocity_jacobian = np.vstack([np.vstack([e_n2D, e_t2D, np.array([zc, -xc])]).T,
         	np.array([0., 0., 1.])])

    # compute end effector velocity in contact frame
    ee_vel_contact_frame = np.linalg.solve(velocity_jacobian, 
    	np.array([contact_vel[0], contact_vel[2], theta_dot]))

    # find normal and tangential displacment
    s = e_t2D[0]*xc + e_t2D[1]*zc 
    d = e_n2D[0]*xc + e_n2D[1]*zc 

    # find angle
    tht = get_hand_orientation_in_base(contact_pose_homog)

    return ee_vel_contact_frame, np.array([d, s, tht])
        
def pivot_xyz_callback(data):
    global pivot_xyz
    pivot_xyz =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]

if __name__ == '__main__':

    rospy.init_node("sliding_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    # initialize globals
    pivot_xyz = None

    # setting up subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_frame", TransformStamped, pivot_xyz_callback)

    # setting up publishers
    generalized_positions_pub = rospy.Publisher('/generalized_positions', 
        Float32MultiArray, queue_size=10)
    generalized_velocities_pub = rospy.Publisher('/generalized_velocities', 
        Float32MultiArray, queue_size=10)

    # define messages
    position_msg, velocity_msg = Float32MultiArray(), Float32MultiArray()

    # make sure subscribers are receiving commands
    print("Waiting for pivot estimate to stabilize")
    while pivot_xyz is None:
        rospy.sleep(0.1)

    print("Starting to publish sliding velocity/position")
    while not rospy.is_shutdown():

        # face_center franka pose
        endpoint_pose_franka = arm.endpoint_pose()

        # face_center list
        endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

        # face center velocity franka        
        endpoint_velocity = arm.endpoint_velocity()

        # face center velocity list
        endpoint_velocity_list = franka_helper.franka_velocity2list(
            endpoint_velocity) 

        # update sliding velocity
        ee_vel_contact_frame, ee_pos_contact_frame = update_sliding_velocity(pivot_xyz[0],
            pivot_xyz[2], endpoint_pose_list, endpoint_velocity_list)

        # update messages
        position_msg.data = ee_pos_contact_frame
        velocity_msg.data = ee_vel_contact_frame

        # publish
        generalized_positions_pub.publish(position_msg)
        generalized_velocities_pub.publish(velocity_msg)
        rate.sleep()    