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



def effort_dict_to_list(effort_dict):  
  
  return np.concatenate([effort_dict['force'], 
    effort_dict['torque']])

if __name__ == '__main__':
    rospy.init_node("path_testing")
    arm = ArmInterface()
    rospy.sleep(0.5)    

    stiffness = [100, 100, 100, 0, 0, 0]
    time_traj, effort_traj = [], []

    # get start pose
    start_pose = arm.endpoint_pose()
    print(start_pose)

    # unload
    pose1 = copy.deepcopy(start_pose)
    pose1['position'][2] += 0.05

    # go to un-loaded pose
    arm.set_cart_impedance_pose(pose1, 
      stiffness=stiffness)
    
    t = time.time()
    while time.time() - t < 2:
      time_traj+= [time.time() - t]
      effort_traj += [effort_dict_to_list(
        arm.endpoint_effort())]
    
    # set target pose to pre-load object
    pose2 = copy.deepcopy(pose1)
    pose2['position'][2] -= 0.15

    # go to pre-loaded pose
    arm.set_cart_impedance_pose(pose2, 
      stiffness=stiffness)    

    # for the next two seconds
    t = time.time()
    while time.time() - t < 2:
      time_traj+= [time.time() - t + 2]
      effort_traj += [effort_dict_to_list(
        arm.endpoint_effort())]

    # set target to move back
    pose3 = copy.deepcopy(pose2)
    pose3['position'][0] -= 0.1

    # go to pose 0.1 cm back
    arm.set_cart_impedance_pose(pose3, 
      stiffness=stiffness)
    
    # for the next two seconds
    t = time.time()
    while time.time() - t < 2:
      time_traj+= [time.time() - t + 4]
      effort_traj += [effort_dict_to_list(
        arm.endpoint_effort())]

    # plot forces
    time_traj = np.array(time_traj)
    effort_traj = np.array(effort_traj)
    fig, ax = plt.subplots(1, 1)
    labels = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
    for i, label in enumerate(labels):
      ax.plot(time_traj, effort_traj[:, i], label=label)
      ax.legend()

    plt.show()