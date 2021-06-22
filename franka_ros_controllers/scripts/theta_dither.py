#!/usr/bin/env python
import rospy
import pdb
import numpy as np

from std_msgs.msg import Float32
from franka_ros_controllers.msg import PbalBarrierFuncCommand

def gravity_torque_callback(data):
    global mgl
    mgl = data.data

if __name__ == '__main__':

    rospy.init_node("barrier_func_commands")
    rate = rospy.Rate(0.2) # in yaml
    rospy.sleep(1.0)


    command_msg = PbalBarrierFuncCommand()


    command_msg.theta = np.pi/8
    command_msg.x = 0.5
    command_msg.s = 0.0
    command_msg.command_flag = 0
    command_msg.mode = -1

    control_command_pub = rospy.Publisher(
        '/barrier_func_control_command', 
        PbalBarrierFuncCommand,
        queue_size=10)

    gravity_torque_sub = rospy.Subscriber("/gravity_torque", 
        Float32, gravity_torque_callback)

    mgl = None
    print("starting theta dither")
    while (mgl is None) and (not rospy.is_shutdown()):
            
            control_command_pub.publish(command_msg)
            command_msg.theta *= -1
            rate.sleep()

    command_msg.theta = 0
    control_command_pub.publish(command_msg)  
    print("theta dither complete")

        


    # terminate rosbags
    # ros_helper.terminate_rosbag()

