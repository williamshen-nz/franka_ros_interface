#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb

import ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
import matplotlib.pyplot as plt

def generalized_velocities_callback(data):
    global generalized_velocities
    generalized_velocities = data

def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_end_effector_frame
    end_effector_wrench_in_end_effector_frame = data

def get_xy_wrench(wrench_list):
    return [wrench_list[0], wrench_list[1], wrench_list[-1]]


if __name__ == '__main__':

    rospy.init_node("friction_coeff_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    rate = rospy.Rate(100)

    # initialize globals
    end_effector_wrench_in_end_effector_frame, generalized_velocities = None, None

    #setting up subscribers
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)
    generalized_velocities_sub = rospy.Subscriber("/generalized_velocities", 
        Float32MultiArray,  generalized_velocities_callback)

    # setting up publisher
    robot_friction_estimate_pub = rospy.Publisher('/robot_friction_estimate', Float32, 
        queue_size=10)

    # make sure subscribers are receiving commands
    print("Waiting for end effector wrench")
    while end_effector_wrench_in_end_effector_frame is None:
        rospy.sleep(0.1)

    print("Waiting for generalized velocities")
    while generalized_velocities is None:
        rospy.sleep(0.1)

    # initialize
    generalized_velocities_list = []
    end_effector_2D_wrench_list = []
    # end_effector_2D_wrench_list_plot_buffer = []
    friction_list = []
    friction_estimate_message = Float32()

    # hyperparameters
    sliding_threshold = 0.03

    # initialize estimate
    friction_estimate = 0.
    num_measurements = 0.
    update_plot_count = 300

    plt.axis([0, 10, 0, 70])

    print("starting force collection")
    while not rospy.is_shutdown():

        # end effector 2D wrench
        end_effector_2D_wrench = get_xy_wrench(ros_helper.wrench_stamped2list(
            end_effector_wrench_in_end_effector_frame))

        # if we are sliding
        if generalized_velocities.data[1] > sliding_threshold:
            
            # if is new a velocity 
            if len(generalized_velocities_list)==0 or \
                generalized_velocities_list[-1][1]!= generalized_velocities.data[1]:
                
                num_measurements+=1. # update counter

                # frition measurement
                friction_measurement=np.abs(end_effector_2D_wrench[1]
                    )/np.abs(end_effector_2D_wrench[0])
                friction_list.append(friction_measurement)

                # update estimate
                friction_estimate=(friction_measurement/num_measurements) + (
                    (num_measurements-1.)/num_measurements)*friction_estimate

                end_effector_2D_wrench_list.append(end_effector_2D_wrench)
                end_effector_2D_wrench_list_plot_buffer.append(end_effector_2D_wrench)
                generalized_velocities_list.append(generalized_velocities.data)

                # if len(end_effector_2D_wrench_list_plot_buffer) > update_plot_count:

                #     end_effector_2D_wrench_list_plot_array = np.abs(
                #         np.array(end_effector_2D_wrench_list_plot_buffer))



                #     plt.scatter(end_effector_2D_wrench_list_plot_array[:,1], 
                #         end_effector_2D_wrench_list_plot_array[:,0])
                #     plt.plot(np.array([0., 8.]), (1/friction_estimate)*np.array([0., 8]), 
                #         color='r')
                #     end_effector_2D_wrench_list_plot_buffer=[]
                #     plt.pause(0.01)

        # publish and sleep
        if num_measurements > 0:
            friction_estimate_message.data = friction_estimate
            robot_friction_estimate_pub.publish(friction_estimate_message)           

        rate.sleep()    
