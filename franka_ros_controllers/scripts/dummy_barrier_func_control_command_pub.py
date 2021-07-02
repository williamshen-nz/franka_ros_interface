#!/usr/bin/env python
import rospy
import pdb
import numpy as np
import json
from std_msgs.msg import String
import time

def prune_command_message(command_msg):

    command_msg_pruned = dict()
    command_flag, mode = command_msg["command_flag"], command_msg["mode"]
    command_msg_pruned["command_flag"] = command_flag
    command_msg_pruned["mode"] = mode

    if command_flag == 0: # absolute move

        if mode == -1:
            command_msg_pruned["theta"] = command_msg["theta"]

        if mode == 0:
            command_msg_pruned["theta"] = command_msg["theta"]
            command_msg_pruned["s"] = command_msg["s"]

        if mode == 1:
            command_msg_pruned["theta"] = command_msg["theta"]
            command_msg_pruned["s"] = command_msg["s"]

        if mode == 2:
            command_msg_pruned["theta"] = command_msg["theta"]
            command_msg_pruned["x_pivot"] = command_msg["x_pivot"]


        if mode == 3:
            command_msg_pruned["theta"] = command_msg["theta"]
            command_msg_pruned["x_pivot"] = command_msg["x_pivot"]

        if mode == 4:
            command_msg_pruned["theta"] = command_msg["theta"]
            command_msg_pruned["xhand"] = command_msg["xhand"]
            command_msg_pruned["zhand"] = command_msg["zhand"]

        if mode == 5:
            command_msg_pruned["xhand"] = command_msg["xhand"]
            command_msg_pruned["zhand"] = command_msg["zhand"]


    if command_flag == 1: # relative move

        if mode == -1:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]

        if mode == 0:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]
            command_msg_pruned["delta_s"] = command_msg["delta_s"]

        if mode == 1:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]
            command_msg_pruned["delta_s"] = command_msg["delta_s"]

        if mode == 2:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]
            command_msg_pruned["delta_x_pivot"] = command_msg["delta_x_pivot"]


        if mode == 3:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]
            command_msg_pruned["delta_x_pivot"] = command_msg["delta_x_pivot"]

        if mode == 4:
            command_msg_pruned["delta_theta"] = command_msg["delta_theta"]
            command_msg_pruned["delta_xhand"] = command_msg["delta_xhand"]
            command_msg_pruned["delta_zhand"] = command_msg["delta_zhand"]

        if mode == 5:
            command_msg_pruned["delta_xhand"] = command_msg["delta_xhand"]
            command_msg_pruned["delta_zhand"] = command_msg["delta_zhand"]

    return command_msg_pruned





if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rospy.sleep(1.0)

    command_msg = String()

    delta_rotate_left = {
        "command_flag" : 1,
        "mode" : -1,
        "delta_theta" : np.pi/8,
        "delta_x_pivot" : 0.0,
        "delta_s" : 0.0,
    }

    delta_rotate_right = {
        "command_flag" : 1,
        "mode" : -1,
        "delta_theta" : -np.pi/8,
        "delta_x_pivot" : 0.0,
        "delta_s" : 0.0,
    }

    delta_slide_robot_left = {
        "command_flag" : 1,
        "mode" : 1,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : -0.01,
    }

    delta_slide_robot_right = {
        "command_flag" : 1,
        "mode" : 0,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.00,
        "delta_s" : 0.01,
    }

    delta_slide_pivot_left = {
        "command_flag" : 1,
        "mode" : 2,
        "delta_theta" : 0,
        "delta_x_pivot" : 0.06,
        "delta_s" : 0.00,
    }

    delta_slide_pivot_right = {
        "command_flag" : 1,
        "mode" : 3,
        "delta_theta" : 0,
        "delta_x_pivot" :- 0.06,
        "delta_s" : 0.00,
    }

    delta_guarded_right = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : -0.05,
        "delta_zhand" : .00,
    }

    delta_guarded_left = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.05,
        "delta_zhand" : .00,
    }

    delta_guarded_down = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.0,
        "delta_zhand" : -.03,
    }

    delta_guarded_up = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0,
        "delta_xhand" : 0.0,
        "delta_zhand" : .03,
    }

    delta_guarded_clockwise = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : -np.pi/6,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_guarded_counterclockwise = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : np.pi/6,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_guarded_static = {
        "command_flag" : 1,
        "mode" : 4,
        "delta_theta" : 0.0,
        "delta_xhand" : 0.0,
        "delta_zhand" : .00,
    }

    delta_flush_right = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : -0.05,
        "delta_zhand" : .00,
    }

    delta_flush_left = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : 0.05,
        "delta_zhand" : .00,
    }

    delta_flush_static = {
        "command_flag" : 1,
        "mode" : 5,
        "delta_xhand" : 0.00,
        "delta_zhand" : .00,
    }

    absolute_rotate_center = {
        "command_flag" : 0,
        "mode" : -1,
        "theta" : 0,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }


    absolute_rotate_left = {
        "command_flag" : 0,
        "mode" : -1,
        "theta" : np.pi/10,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }

    absolute_rotate_right = {
        "command_flag" : 0,
        "mode" : -1,
        "theta" : -np.pi/10,
        "x_pivot" : 0.0,
        "s" : 0.0,
    }

    command_msg_dict = { 
        "command_flag" : 1,
        "mode" : -1,
        "theta" : 0,
        "x_pivot" : 0.5,
        "s" : -0.01,
        "delta_theta" : 0,
        "delta_x_pivot" : -0.05,
        "delta_s" : .00,
        "xhand" : 0,
        "zhand" : 0,
        "theta" : 0,
        "delta_xhand" : -0.1,
        "delta_zhand" : .00,
    }

    #command_msg_dict = absolute_rotate_right
    #command_msg_dict = absolute_rotate_left
    #command_msg_dict = absolute_rotate_center
    #command_msg_dict = delta_rotate_left
    #command_msg_dict = delta_rotate_right
    #command_msg_dict = delta_slide_robot_left
    #command_msg_dict = delta_slide_robot_right
    #command_msg_dict = delta_slide_pivot_left
    #command_msg_dict = delta_slide_pivot_right


    #command_msg_dict = delta_flush_static
    #command_msg_dict = delta_guarded_static
    #command_msg_dict = delta_guarded_right
    #command_msg_dict = delta_guarded_left
    #command_msg_dict = delta_guarded_down
    #command_msg_dict = delta_guarded_up
    #command_msg_dict = delta_guarded_clockwise
    #command_msg_dict = delta_guarded_counterclockwise
    #command_msg_dict = delta_flush_right
    #command_msg_dict = delta_flush_left

    message_queue= [
        absolute_rotate_left,
        delta_slide_robot_left,
        delta_slide_pivot_right,
        absolute_rotate_right,
        delta_slide_robot_right,
        delta_slide_pivot_left
    ]

    message_queue = [
        absolute_rotate_left,
        absolute_rotate_right
    ]


    command_msg_dict = delta_flush_static
    command_msg_dict_pruned = prune_command_message(command_msg_dict)
    command_msg_string_pruned = json.dumps(command_msg_dict_pruned)


    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        String,
        queue_size=10)


    published = False
    while (not published) and (not rospy.is_shutdown()):
        
        print(control_command_pub.get_num_connections())
        if control_command_pub.get_num_connections() == 1:

            command_msg.data = command_msg_string_pruned 
     
            control_command_pub.publish(command_msg)
            published = True

    #time.sleep(3)

    #for count in (1,5):
    #    for message in message_queue:
    #        command_msg_dict = message
    #        command_msg_dict_pruned = prune_command_message(command_msg_dict)
    #        command_msg_string_pruned = json.dumps(command_msg_dict_pruned)
    #        command_msg.data = command_msg_string_pruned 
    #        control_command_pub.publish(command_msg)
    #        time.sleep(15)

    #command_msg_dict = absolute_rotate_center
    #command_msg_dict_pruned = prune_command_message(command_msg_dict)
    #command_msg_string_pruned = json.dumps(command_msg_dict_pruned)
    #command_msg.data = command_msg_string_pruned 
    #control_command_pub.publish(command_msg)






        
