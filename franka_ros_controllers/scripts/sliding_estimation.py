#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb
import copy
import time

import ros_helper
import franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray, Bool, Int32
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt


def get_hand_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def update_sliding_velocity(x0, z0, contact_pose, contact_vel, 
    torque_boundary_flag, ee_pos_contact_frame_old, LCONTACT, RATE, axs):

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

    # xc and zc (with pivot at 0,0)
    xc, zc = contact_pose[0] - x0, contact_pose[2] - z0

    if torque_boundary_flag == -1 or ee_pos_contact_frame_old is None:

        # 2-D angular velocity
        theta_dot = contact_vel[4]

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
        tht_hand = tht

        # generalized position
        ee_pos_contact_frame = np.array([d, s, tht])

        # p1x, p1z = -d*np.sin(tht), -d*np.cos(tht)
        # p2x, p2z = p1x - s * np.cos(tht), p1z + s * np.sin(tht)
        # p3x, p3z = p2x + 0.5 * LCONTACT * e_t2D[0], p2z + 0.5 * LCONTACT * e_t2D[1]
        # p4x, p4z = p2x - 0.5 * LCONTACT * e_t2D[0], p2z - 0.5 * LCONTACT * e_t2D[1]

        # axs.clear()
        # axs.plot([0, p1x, p2x, p3x, p4x], 
        #     [0, p1z, p2z, p3z, p4z], 'k')
        # axs.scatter([p2x], [p2z], color='r')
        # axs.scatter([xc], [zc], color='y')    

    elif torque_boundary_flag == 0:
        print("Not in contact")
        ee_pos_contact_frame = copy.deepcopy(ee_pos_contact_frame_old)
        ee_vel_contact_frame = np.zeros_like(ee_pos_contact_frame)
        tht_hand = ee_pos_contact_frame[-1]

    elif torque_boundary_flag == 1 or torque_boundary_flag == 2:

        d_old, s_old, theta_old = ee_pos_contact_frame_old[0
            ], ee_pos_contact_frame_old[1], ee_pos_contact_frame_old[2]

        if torque_boundary_flag == 1:

            xcontact = xc + 0.5 * LCONTACT * e_t2D[0]
            zcontact = zc + 0.5 * LCONTACT * e_t2D[1]

        if torque_boundary_flag == 2:
            
            xcontact = xc - 0.5 * LCONTACT * e_t2D[0]
            zcontact = zc - 0.5 * LCONTACT * e_t2D[1]

        # angle of hand
        tht_hand = get_hand_orientation_in_base(contact_pose_homog)

        # length of vector from pivot to (xcontact, zcontact)
        lsquared = xcontact ** 2 + zcontact ** 2

        # two possible values for sliding position of contact point
        if lsquared < d_old**2:
            sp_contact = 0
            sm_contact = 0
        else:
            sp_contact = np.sqrt(lsquared - d_old ** 2)
            sm_contact = -np.sqrt(lsquared - d_old ** 2)

        # two possible values for angle of object
        thtp = np.arctan2(xcontact, zcontact) + np.arctan2(sp_contact, 
            np.abs(d_old))
        thtm = np.arctan2(xcontact, zcontact) + np.arctan2(sm_contact, 
            np.abs(d_old))

        # pick correct value of theta and s
        if torque_boundary_flag == 1:

            if thtp > tht_hand and thtm < tht_hand:
                s_contact = sp_contact
                tht, s = thtp, s_contact - 0.5 * LCONTACT
            elif thtp < tht_hand and thtm > tht_hand:
                s_contact = sm_contact
                tht, s = thtm, s_contact - 0.5 * LCONTACT
            else: 
                thtp_err = np.abs(thtp - theta_old)
                thtm_err = np.abs(thtm - theta_old)

                if thtp_err < thtm_err:
                    s_contact = sp_contact
                    tht, s = thtp, s_contact - 0.5 * LCONTACT
                else:
                    s_contact = sm_contact
                    tht, s = thtm, s_contact - 0.5 * LCONTACT

        if torque_boundary_flag == 2:

            if thtp < tht_hand and thtm > tht_hand:
                s_contact = sp_contact
                tht, s = thtp, s_contact + 0.5 * LCONTACT
            elif thtp > tht_hand and thtm < tht_hand:
                s_contact = sm_contact
                tht, s = thtm, s_contact + 0.5 * LCONTACT
            else: 
                thtp_err = np.abs(thtp - theta_old)
                thtm_err = np.abs(thtm - theta_old)

                if thtp_err < thtm_err:
                    s_contact = sp_contact
                    tht, s = thtp, s_contact + 0.5 * LCONTACT
                else:
                    s_contact = sm_contact
                    tht, s = thtm, s_contact + 0.5 * LCONTACT

        # generalized position
        ee_pos_contact_frame = np.array([d_old, s, tht])

        ee_vel_contact_frame = (ee_pos_contact_frame - 
            ee_pos_contact_frame_old)/RATE

        # p1x, p1z = -d_old*np.sin(tht), -d_old*np.cos(tht)
        # p2x, p2z = p1x - s * np.cos(tht), p1z + s * np.sin(tht)
        # p3x, p3z = p1x - s_contact *  np.cos(tht), p1z + s_contact * np.sin(tht)

        # if torque_boundary_flag == 1:
        #     multiplier = -1
        # elif torque_boundary_flag == 2:
        #     multiplier = 1

        # p4x = p3x + multiplier * 0.5 * LCONTACT * e_t2D[0]
        # p4z = p3z + multiplier * 0.5 * LCONTACT * e_t2D[1]

        # p5x = p3x + multiplier * LCONTACT * e_t2D[0]
        # p5z = p3z + multiplier * LCONTACT * e_t2D[1]

        # axs.clear()
        # axs.plot([0, p1x, p2x, p3x, p4x, p5x], 
        #     [0, p1z, p2z, p3z, p4z, p5z], 'k')
        # axs.scatter([p2x, p4x], [p2z, p4z], color='r')
        # axs.scatter([xcontact, xc], [zcontact, zc], color='y')

    else:
        raise RuntimeError("incorrect torque_boundary_flag value")


    return ee_vel_contact_frame, ee_pos_contact_frame, tht_hand, axs
        
def pivot_xyz_callback(data):
    global pivot_xyz
    pivot_xyz =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def torque_cone_boundary_flag_callback(data):
    global torque_cone_boundary_flag
    torque_cone_boundary_flag = data.data


if __name__ == '__main__':

    rospy.init_node("sliding_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    RATE = 100.
    LCONTACT = 0.065
    rate = rospy.Rate(RATE)

    # initialize globals
    pivot_xyz = None
    torque_boundary_boolean = None
    torque_cone_boundary_flag = None
    ee_pos_contact_frame_old = None

    # setting up subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_frame", TransformStamped, pivot_xyz_callback)
    torque_cone_boundary_test_sub = rospy.Subscriber("/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)
    torque_cone_boundary_flag_sub = rospy.Subscriber("/torque_cone_boundary_flag", 
        Int32,  torque_cone_boundary_flag_callback)

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

    print("Waiting for torque boundary check")
    while torque_boundary_boolean is None:
        pass

    print("Waiting for torque boundary check")
    while torque_cone_boundary_flag is None:
        pass

    # fig, axs = plt.subplots(1,1)
    axs = None
    # plt.show()

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

        # axs.invert_xaxis()
        # update sliding velocity
        ee_vel_contact_frame, ee_pos_contact_frame, tht_hand, axs = update_sliding_velocity(
            pivot_xyz[0], pivot_xyz[2], endpoint_pose_list,
            endpoint_velocity_list, torque_cone_boundary_flag, 
            ee_pos_contact_frame_old, LCONTACT, RATE, axs)
        # axs.axis('equal')
        # axs.set_xlim([0.1, -0.1])
        # axs.set_ylim([0.0, 0.2])
        # plt.pause(0.01)

        # input("Press enter to continue")

        # update messages
        position_msg.data = ee_pos_contact_frame
        velocity_msg.data = ee_vel_contact_frame
        ee_pos_contact_frame_old = ee_pos_contact_frame

        # publish
        generalized_positions_pub.publish(position_msg)
        generalized_velocities_pub.publish(velocity_msg)
        rate.sleep()