#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import WrenchStamped
import netft_rdt_driver.srv as srv
from ros_helper import (unit_pose, list2pose_stamped, pose_stamped2list,
                               convert_reference_frame, quat2list, 
                               lookupTransform, wrenchstamped_2FT, rotate_wrench, 
                               wrench_reference_point_change)

# ft sensor topic
ft_wrench_in_ft_sensor_frame = "/netft/netft_data"

def zero_ft_sensor():
    rospy.wait_for_service('/netft/zero', timeout=0.5)
    zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
    zero_ft()

def force_callback(data):
    global ft_wrench_in_ft_sensor
    ft_wrench_in_ft_sensor = data


if __name__ == '__main__':

    # initialize node
    rospy.init_node('ft_sensor_in_base_frame', anonymous=True)
    rate = rospy.Rate(100.)

    # Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # ft sensor in the hand frame
    (ft_sensor_in_end_effector_trans, ft_sensor_in_end_effector_rot) = \
        lookupTransform('/ft_sensor', '/panda_hand', listener)

    # Subscribe to ft data
    ft_wrench_in_ft_sensor = None
    ft_wrench_in_ft_sensor_frame_sub = rospy.Subscriber(
        ft_wrench_in_ft_sensor_frame, WrenchStamped, force_callback, queue_size=1)

    # Define rostopic publishers
    ft_sensor_in_base_frame_pub = rospy.Publisher('/ft_sensor_in_base_frame', 
        WrenchStamped, queue_size = 10)
    ft_sensor_in_end_effector_frame_pub = rospy.Publisher('/ft_sensor_in_end_effector_frame', 
        WrenchStamped, queue_size = 10)
    end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
        '/end_effector_sensor_in_end_effector_frame', WrenchStamped, queue_size = 10)
    end_effector_sensor_in_base_frame_pub = rospy.Publisher(
        '/end_effector_sensor_in_base_frame', WrenchStamped, queue_size = 10)


    # wait for ft data
    while ft_wrench_in_ft_sensor is None:
        pass
    
    # zero sensor
    zero_ft_sensor()
    print("Zeroing sensor")

    # Run node at rate
    while not rospy.is_shutdown():

        # ft sensor pose in base frame
        (ft_sensor_in_base_trans, ft_sensor_in_base_rot) = \
            lookupTransform('/ft_sensor', 'base', listener)
        ft_sensor_in_base_pose = list2pose_stamped(ft_sensor_in_base_trans 
            + ft_sensor_in_base_rot, frame_id="base")

        # panda hand pose in base frame
        (panda_hand_in_base_trans, panda_hand_in_base_rot) = \
            lookupTransform('/panda_hand', 'base', listener)
        panda_hand_in_base_pose = list2pose_stamped(panda_hand_in_base_trans 
            + panda_hand_in_base_rot, frame_id="base")

        # ft sensor pose in end effector frame
        (ft_sensor_in_end_effector_trans, ft_sensor_end_effector_in_base_rot) = \
            lookupTransform('/ft_sensor', '/panda_hand', listener)
        ft_sensor_in_end_effector_pose = list2pose_stamped(ft_sensor_in_end_effector_trans 
            + ft_sensor_end_effector_in_base_rot, frame_id="/panda_hand")
 
        # ft wrench in base frame
        ft_wrench_in_base = rotate_wrench(ft_wrench_in_ft_sensor, 
            ft_sensor_in_base_pose)
        ft_wrench_in_base.header.frame_id = 'base'

        # ft wrench in end-effector frame
        ft_wrench_in_end_effector = rotate_wrench(ft_wrench_in_ft_sensor, 
            ft_sensor_in_end_effector_pose)
        ft_wrench_in_end_effector.header.frame_id = "/panda_hand"

        # end effector wrench in end effector frame
        end_effector_wrench_in_end_effector = wrench_reference_point_change(
            ft_wrench_in_end_effector, ft_sensor_in_end_effector_trans)
        end_effector_wrench_in_end_effector.header.frame_id = "/panda_hand"

        # end effector wrench in base frame
        end_effector_wrench_in_base = rotate_wrench(end_effector_wrench_in_end_effector, 
            panda_hand_in_base_pose)
        end_effector_wrench_in_base.header.frame_id = "base"


        # publish and sleep
        ft_sensor_in_base_frame_pub.publish(ft_wrench_in_base)
        ft_sensor_in_end_effector_frame_pub.publish(ft_wrench_in_end_effector)
        end_effector_sensor_in_end_effector_frame_pub.publish(end_effector_wrench_in_end_effector)
        end_effector_sensor_in_base_frame_pub.publish(end_effector_wrench_in_base)
        rate.sleep()