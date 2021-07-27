import rospy
import pdb
import numpy as np
import json
from std_msgs.msg import String
import time
import models.ros_helper as ros_helper



if __name__ == '__main__':

    rospy.init_node("rospy_recording_node")
    rate = rospy.Rate(30)

    # set up rosbag
    rostopic_list = ["/camera/color/image_raw/compressed",
                     "/face_contact_center_pose_in_world_frame_publisher",
                     "/obj_apriltag_pose_in_world_from_camera_publisher",
                     "/generalized_positions",
                     "/end_effector_sensor_in_base_frame",
                     "/com_ray",
                     "/pivot_marker",
                     "/gravity_torque",
                     "/external_wrench_in_pivot",
                     "/robot_friction_estimate"]

    ros_helper.initialize_rosbag(rostopic_list, exp_name="blind_pivot_triangle")

    print('starting recording node')
    
    while not rospy.is_shutdown():

        rate.sleep()

    # terminate rosbags
    ros_helper.terminate_rosbag()
