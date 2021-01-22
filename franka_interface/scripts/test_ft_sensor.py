#!/usr/bin/env python

import pdb
import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import EmptyResponse, Empty as EmptyServiceMsg
import netft_rdt_driver.srv as srv


def zero_ft_sensor():
	rospy.wait_for_service('/netft/zero', timeout=0.5)
	zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
	zero_ft()

def force_callback(data):
    global ft_data    
    ft_data = data

if __name__ == '__main__':

	rospy.init_node('ft_sensor_test', anonymous=True)


	ft_data = None	
	ft_sub = rospy.Subscriber('/netft/netft_data', WrenchStamped, force_callback, 
		queue_size=1)
	
	# wait for ft data
	while ft_data is None:
		pass
	
	# zero sensor
	zero_ft_sensor()

	t0 = rospy.Time.now().to_sec()
	while True:
		time = ft_data.header.stamp.secs-t0;
		fxyz_list = [ft_data.wrench.force.x, ft_data.wrench.force.y,
			ft_data.wrench.force.z]
		print("Time: " + str(time))
		print("Force: " + str(fxyz_list))
		rospy.sleep(1.0)



