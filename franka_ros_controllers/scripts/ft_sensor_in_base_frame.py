#!/usr/bin/env python

import rospy
import tf
import pdb

from geometry_msgs.msg import WrenchStamped
import netft_rdt_driver.srv as srv
from ros_helper import (unit_pose, list2pose_stamped, pose_stamped2list,
                               convert_reference_frame, quat2list, 
                               lookupTransform, wrenchstamped_2FT, transform_wrench)

# ft sensor topic
ft_sensor_in_ft_sensor_frame = "/netft/netft_data"

def zero_ft_sensor():
    rospy.wait_for_service('/netft/zero', timeout=0.5)
    zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
    zero_ft()

def force_callback(data):
    global wrench_in_ft_sensor
    wrench_in_ft_sensor = data


if __name__ == '__main__':

    # initialize node
    rospy.init_node('ft_sensor_in_base_frame', anonymous=True)
    rate = rospy.Rate(100.)

    # Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # # ft_sensor in panda_hand frame
    (ft_sensor_in_panda_hand_trans, ft_sensor_in_panda_hand_rot) = \
        lookupTransform('/ft_sensor', 'base', listener)
    ft_sensor_in_panda_hand_pose = list2pose_stamped(ft_sensor_in_panda_hand_trans 
        + ft_sensor_in_panda_hand_rot, frame_id="panda_hand")

    # base frame in base frame
    base_in_base_pose = unit_pose()

    # Subscribe to ft data
    wrench_in_ft_sensor = None
    ft_sensor_in_ft_sensor_frame_sub = rospy.Subscriber(ft_sensor_in_ft_sensor_frame, 
        WrenchStamped, force_callback, queue_size=1)

    # Define rostopic publishers
    ft_sensor_in_base_frame_pub = rospy.Publisher('/ft_sensor_in_base_frame', 
        WrenchStamped, queue_size = 10)

    # wait for ft data
    while wrench_in_ft_sensor is None:
        pass
    
    # zero sensor
    zero_ft_sensor()

    # Run node at rate
    while not rospy.is_shutdown():

        # tooltip pose in base frame
        (ft_sensor_in_base_trans, ft_sensor_in_base_rot) = \
            lookupTransform('/ft_sensor', 'base', listener)
        ft_sensor_in_base_pose = list2pose_stamped(ft_sensor_in_base_trans 
            + ft_sensor_in_base_rot, frame_id="base")
 
        # ft wrench in base frame
        wrench_in_base = transform_wrench(wrench_in_ft_sensor, 
            ft_sensor_in_base_pose)
        wrench_in_base.header.frame_id = 'panda_hand'

        # publish and sleep
        ft_sensor_in_base_frame_pub.publish(wrench_in_base)
        rate.sleep()