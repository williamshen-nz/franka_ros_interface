#!/usr/bin/env python
import rospy
import pdb
import numpy as np
from franka_ros_controllers.msg import PbalBarrierFuncCommand





if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rospy.sleep(1.0)


    command_msg = PbalBarrierFuncCommand()

    command_msg.command_flag = 0
    command_msg.mode = -1

    command_msg.theta = 0
    command_msg.x = 0.5
    command_msg.s = 0.0

    command_msg.delta_theta = np.pi/20
    command_msg.delta_x = -0.05
    command_msg.delta_s = -0.01

    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        PbalBarrierFuncCommand,
        queue_size=10)


    published = False
    while not published:
        
        print(control_command_pub.get_num_connections())
        if control_command_pub.get_num_connections() == 1:
        
            control_command_pub.publish(command_msg)
            published = True
    


        


    # terminate rosbags
    # ros_helper.terminate_rosbag()

