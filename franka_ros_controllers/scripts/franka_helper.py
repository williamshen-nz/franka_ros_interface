import numpy as np

def franka_pose2list(arm_pose):
    '''
    arm pose is a franka pose
    output is a list
    '''
    position_list = arm_pose['position'].tolist()
    orientation_list = [arm_pose['orientation'].x,
                               arm_pose['orientation'].y,
                               arm_pose['orientation'].z,
                               arm_pose['orientation'].w]
    return position_list + orientation_list

def franka_orientation2list(orientation):
    orientation_list = [orientation.x, orientation.y,
                               orientation.z,
                               orientation.w]
    return orientation_list

def franka_velocity2list(arm_vel):
    '''
    arm vel is a franka vel
    output is a list
    '''
    return arm_vel['linear'].tolist() + arm_vel['angular'].tolist() 