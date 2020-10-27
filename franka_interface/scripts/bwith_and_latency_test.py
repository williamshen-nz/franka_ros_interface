#! /usr/bin/env python


"""
 @info: 
   commands robot to move to neutral pose

"""

import rospy
import copy
import IPython
import numpy as np
import time
import matplotlib.pyplot as plt
import pdb
from franka_interface import ArmInterface 
# from franka_tools import CollisionBehaviourInterface


if __name__ == '__main__':
    # This seems to work fairly well, sometimes the gripper
    # is backdriven instead of the the flush operation working

    rospy.init_node("bwith_testing")
    arm = ArmInterface()
    rospy.sleep(0.5)

    # switch controllers
    print("Switching controllers...")
    arm.switchToController(arm._ctrl_manager.joint_trajectory_controller)
    rospy.sleep(1.0)

    # get start pose
    start_pose = arm.joint_angles()
    # print(start_pose)

    # set target pose
    goal_pose = copy.deepcopy(start_pose)
    goal_pose['panda_joint1'] += 0.3
    

    # go to target pose
    for i in range(4):
      arm.set_joint_positions(goal_pose)  
    # rospy.sleep(1.0)
  

    # time_traj, joint_traj = [], []
    # t = time.time()
    # while time.time() - t < 2:
    #   time_traj+= [time.time() - t]
    #   joint_traj += [arm.convertToList(arm.joint_angles())]
    
    # time_traj = np.array(time_traj)
    # joint_traj = np.array(joint_traj)
    # fig, ax = plt.subplots(1, 1)
    # labels = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    # for i, label in enumerate(labels):
    #   ax.plot(time_traj, joint_traj[:, i], label=label)
    #   ax.legend()

    # plt.show()