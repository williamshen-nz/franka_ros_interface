#! /usr/bin/env python


"""
 @info: 
   commands robot to move to neutral pose

"""

import rospy
import copy
import IPython
import numpy as np
from franka_interface import ArmInterface 



def effort_dict_to_list(effort_dict):  
  
  return np.concatenate([effort_dict['force'], 
    effort_dict['torque']])

if __name__ == '__main__':
    rospy.init_node("path_testing")
    arm = ArmInterface()
    rospy.sleep(0.5)    

    IPython.embed()