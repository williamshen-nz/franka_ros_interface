#!/usr/bin/env python
# estimates pivot location and publishes pivot location as Marker and Point
# subscribes to end_effector pose 

import numpy as np
import tf.transformations as tfm
import rospy

import ros_helper
import franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def initialize_marker():
    marker_message = Marker()
    marker_message.header.frame_id = "base"
    marker_message.header.stamp = rospy.get_rostime()
    marker_message.type = Marker.SPHERE
    marker_message.scale.x=.02
    marker_message.scale.y=.02
    marker_message.scale.z=.02
    marker_message.color.a=1.0
    marker_message.color.r=0
    marker_message.color.g=1
    marker_message.color.b=0
    marker_message.lifetime.secs=1.0
    return marker_message

def update_marker_pose(marker_pose, marker_message):
    marker_message.header.stamp = marker_pose.header.stamp
    marker_message.pose = marker_pose.pose

def get_2D_normal(random_pose_array):

    nx_list = []
    nz_list = []

    for pose_array in random_pose_array:
        pose_stamped = ros_helper.list2pose_stamped(pose_array.tolist())
        pose_homog = ros_helper.matrix_from_pose(pose_stamped)
        
        # 2-D unit contact normal in world frame
        e_n = pose_homog[:3, 0]
        n2D = e_n[[0,2]]
        e_n2D = n2D/np.sqrt(np.sum(n2D ** 2))

        nx_list.append(e_n2D[0])
        nz_list.append(e_n2D[1])

    return np.array(nx_list), np.array(nz_list)
       

def update_center_of_rotation_estimate(pose_list):

    pose_array = np.array(pose_list) # TODO: this should always be an array

    # pull out the x and z coordinates of this triangle 
    random_x = pose_array[:, 0]
    random_z = pose_array[:, 2]

    # pull out 2D unit contact normal in world frame
    nx_array, nz_array = get_2D_normal(pose_array)

    # build Ax = b 
    b = nx_array * random_x + nz_array * random_z
    A = np.vstack([nx_array, nz_array, np.ones_like(nx_array)]).T

    # solve Ax = b to find COR (x = [x0, y0, C])
    Coeff_Vec = np.linalg.lstsq(A,b)[0]

    # extract the center location for the three points chosen
    x0=Coeff_Vec[0]
    z0=Coeff_Vec[1]

    return x0,z0

if __name__ == '__main__':

    rospy.init_node("pivot_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    endpoint_pose_all = []

    # set up pivot Point publisher
    pivot_xyz_pub = rospy.Publisher('/pivot_xyz', PointStamped, queue_size=10)      

    # set up pivot marker publisher
    marker_message = initialize_marker()
    pivot_marker_pub = rospy.Publisher('/pivot_marker', Marker, queue_size=10)      

    # intialize pivot estimate
    current_pose = arm.endpoint_pose()      # original pose of robot
    x0=None
    z0=None

    # hyper parameters
    Nbatch = 250             # max number of datapoints for estimation
    update_length = 50       # number of good points before update/publish
    diff_threshold = 0.001   # threshold for new datapoints

    print('starting estimation loop')
    while not rospy.is_shutdown():

        # face_center franka pose
        endpoint_pose_franka = arm.endpoint_pose()

        # face_center list
        endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

        # append if empty
        if not endpoint_pose_all:
            endpoint_pose_all.append(endpoint_pose_list)

        # current_quaternion
        endpoint_quat = franka_helper.franka_orientation2list(endpoint_pose_franka['orientation'])

        # last quaternion in list
        endpoint_quat_old = endpoint_pose_all[-1][3:]

        # difference quaternion
        diff_quat = tfm.quaternion_multiply(endpoint_quat, 
                tfm.quaternion_inverse(endpoint_quat_old))

        # diff angle
        diff_angle = 2 * np.arccos(diff_quat[-1])

        # if measured pose is new
        if np.abs(diff_angle) > diff_threshold:
            
            # append to list
            endpoint_pose_all.append(endpoint_pose_list)

            # make list a FIFO buffer of length Nbatch
            if len(endpoint_pose_all) > Nbatch:
                endpoint_pose_all.pop(0)

            # and update
            if len(endpoint_pose_all) > update_length:

                # update center of rotation estimate
                x0, z0 = update_center_of_rotation_estimate(endpoint_pose_all)

                # update maker for center of rotation
                pivot_pose = ros_helper.list2pose_stamped([x0,endpoint_pose_all[-1][1],z0,0,0,0,1])
                update_marker_pose(pivot_pose, marker_message)

        # only publish after estimate has settled
        if len(endpoint_pose_all) > update_length:
            pivot_xyz_pub.publish(ros_helper.list2point_stamped([x0, endpoint_pose_all[-1][1], z0]))
            pivot_marker_pub.publish(marker_message)
        
        rate.sleep()    



    