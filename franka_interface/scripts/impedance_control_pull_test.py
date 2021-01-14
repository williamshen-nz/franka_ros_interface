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
from franka_tools import CollisionBehaviourInterface



def effort_dict_to_list(effort_dict):  
  
  return np.concatenate([effort_dict['force'], 
    effort_dict['torque']])

if __name__ == '__main__':
    rospy.init_node("path_testing")
    arm = ArmInterface()
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)    

    # set collision behavior
    print("Setting collision behaviour")
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0] # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0] # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)


    stiffness = [300, 300, 300, 30, 30, 30]
    time_traj, effort_traj = [], []

    # get start pose
    start_pose = arm.endpoint_pose()
    print(start_pose)

    # unload
    # unloaded_pose = copy.deepcopy(start_pose)
    # unloaded_pose['position'][2] += 0.05

    # # go to un-loaded pose
    # arm.set_cart_impedance_pose(pose1, 
    #   stiffness=stiffness)
    
    # t = time.time()
    # while time.time() - t < 2:
    #   time_traj+= [time.time() - t]
    #   effort_traj += [effort_dict_to_list(
    #     arm.endpoint_effort())]
    
    # set target pose to pre-load object
    pre_load_pose = copy.deepcopy(start_pose)
    pre_load_pose['position'][2] -= 0.1

    # go to pre-loaded pose
    arm.set_cart_impedance_pose(pre_load_pose, 
      stiffness=stiffness)    

    # # for the next two seconds
    # t = time.time()
    # while time.time() - t < 2:
    #   time_traj+= [time.time() - t + 2]
    #   effort_traj += [effort_dict_to_list(
    #     arm.endpoint_effort())]

    # # set target to move back
    # pose3 = copy.deepcopy(pose2)
    # pose3['position'][0] -= 0.1

    # # go to pose 0.1 cm back
    # arm.set_cart_impedance_pose(pose3, 
    #   stiffness=stiffness)
    
    # # for the next two seconds
    # t = time.time()
    # while time.time() - t < 2:
    #   time_traj+= [time.time() - t + 4]
    #   effort_traj += [effort_dict_to_list(
    #     arm.endpoint_effort())]

    # # plot forces
    # time_traj = np.array(time_traj)
    # effort_traj = np.array(effort_traj)
    # fig, ax = plt.subplots(1, 1)
    # labels = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
    # for i, label in enumerate(labels):
    #   ax.plot(time_traj, effort_traj[:, i], label=label)
    #   ax.legend()

    plt.show()