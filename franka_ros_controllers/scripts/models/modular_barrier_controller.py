import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6
import pdb

from pbal_helper import PbalHelper


class ModularBarrierController(object):
	'''
	everything is in the contact frame unless specified otherwise
	contact_wrench is the wrench the robot applies TO the object
	'''

	def __init__(self, param_dict):

	    # create pbal kinematics helper
        self.pbal_helper = PbalHelper(param_dict = param_dict['obj_params'])

		# set objective function parameters
		self.set_objective_function_paramater_values(param_dict = param_dict)

		# set constraint function parameters
		self.set_constraint_function_parameter_values(param_dict = param_dict)

		# set maximum normal force robot can apply
		self.Nmax = param_dict['Nmax']


    def update_controller(self, mode, theta_hand, contact_wrench,
    	l_hand = None,
    	s_hand = None,
    	err_theta_pivot = None,     	
    	err_s_pivot = None,
    	err_x_pivot = None,
    	err_theta_free = None,
    	err_xhand_free = None,
    	err_zhand_free = None):

    	# reset state errors
    	self.reset_state_errors()

    	# update pose variables
    	self.theta_hand, self.l_hand, self.s_hand = theta_hand, l_hand, s_hand

    	# update_mode 
    	self.mode = mode

    	# update contact wrench
    	self.contact_wrench = contact_wrench

    	# update contact_pose
    	self.contact_pose = contact_pose

    	if mode == -1:  # line/line stick and point/line stick
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
    			(contact_wrench[0] - 0.5 * self.Nmax)/self.N_scale_pivot)

    	if mode == 0:	# line/line slide +  and point/line stick
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_s_pivot = (2./np.pi) * np.arctan(
    			err_s_pivot/self.s_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
    			(contact_wrench[0] - 0.5 * self.Nmax)/self.N_scale_pivot)

    	if mode == 1:	# line/line slide -  and point/line stick
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_s_pivot = (2./np.pi) * np.arctan(
    			err_s_pivot/self.s_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
    			(contact_wrench[0] - 0.5 * self.Nmax)/self.N_scale_pivot)

    	if mode == 2: 	# line/line stick  and point/line slide +
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_x_pivot = (2./np.pi) * np.arctan(
    			err_x_pivot/self.x_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
    			(contact_wrench[0] - 0.5 * self.Nmax)/self.N_scale_pivot)

    	if mode == 3:	# line/line stick  and point/line slide -
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_x_pivot = (2./np.pi) * np.arctan(
    			err_x_pivot/self.theta_x_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
    			(contact_wrench[0] - 0.5 * self.Nmax)/self.N_scale_pivot)

    	if mode == 4:	# free move
    		self.err_theta_free = (2./np.pi) * np.arctan(
    			err_theta_free/self.theta_scale_free)
    		self.err_xhand_free = (2./np.pi) * np.arctan(
    			err_xhand_free/self.xhand_scale_free)
    		self.err_zhand_free = (2./np.pi) * np.arctan(
    			err_zhand_free/self.zhand_scale_free)


    def friction_left_robot_pivot_constraint(self):



    	        # extract parameters
        mu_c, mu_g, lc = self.pbal_helper.mu_contact, \
            self.pbal_helper.mu_ground, self.pbal_helper.l_contact

        # compute pivot constraints
        contact2robot = self.pbal_helper.contact2robot(
            contact_pose)
        Aiq_pivot_robot = np.array([[1., mu_g, 0.], # friction 1
            [-1., mu_g, 0], # friction 2
            [0., 1., 0.]]) # normal force pivot positive
        Aiq_pivot_contact = np.dot(Aiq_pivot_robot, contact2robot)

        # measured wrench is in contact frame
        biq = np.array([0., 0., -self.torque_margin, -self.torque_margin,
            Nmax, 0., 0., 0.])
        Aiq = np.array([[-mu_c, 1., 0.]/np.sqrt(1 + mu_c ** 2), # friction robot 1
            [-mu_c, -1., 0.]/np.sqrt(1 + mu_c ** 2), # friction robot 2
            [-lc / 2., 0., 1.], # torque 1
            [-lc / 2., 0., -1.], # torque 2
            [1., 0., 0.]]) # normal force robot

        Aiq = np.vstack([Aiq, Aiq_pivot_contact]) # pivot friction 

        return np.dot(Aiq, measured_wrench) - biq, Aiq, biq




    def reset_state_errors(self):
    	
    	self.err_theta_pivot = None
    	self.err_s_pivot = None
    	self.err_x_pivot = None
    	self.err_N_pivot = None

    	self.err_theta_free = None
    	self.err_xhand_free = None
    	self.err_zhand_free = None

    def set_objective_function_paramater_values(self, param_dict):

        # objective function parameters: theta, line/line plus point/line
        self.K_theta_pivot = param_dict['K_theta_pivot']
        self.theta_scale_pivot = params_dict['theta_scale_pivot']
        self.concavity_theta_pivot = param_dict['concavity_theta_pivot']
    
    	# objective function parameters: s, line/line plus point/line
        self.K_s_pivot = param_dict['K_s']
        self.s_scale_pivot = param_dict['s_scale_pivot']
        self.concavity_s = param_dict['concavity_s']
    
    	# objective function parameters: x, line/line plus point/line
        self.K_x_pivot = param_dict['K_x_pivot']
        self.x_scale_pivot = param_dict['x_scale_pivot']
        self.concavity_x_pivot = param_dict['concavity_x_pivot']

        # objective function parameters: N, line/line plus point/line
        self.K_N_pivot = param_dict['K_N_pivot']
        self.N_scale_pivot = param_dict['N_scale_pivot']
        self.concavity_N_pivot = param_dict['concavity_N_pivot']
    
        # objective function parameters: theta, free
        self.K_theta_free = param_dict['K_theta_free']
        self.theta_scale_free = params_dict['theta_scale_free']
        self.concavity_theta_free = param_dict['concavity_theta_free']

        # objective function parameters: x-hand, free
        self.K_xhand_free = param_dict['K_xhand_free']
        self.xhand_scale_free = params_dict['xhand_scale_free']
        self.concavity_xhand_free = param_dict['concavity_xhand_free']

        # objective function parameters: z-hand, free
        self.K_zhand_free = param_dict['K_zhand_free']
        self.zhand_scale_free = params_dict['zhand_scale_free']
        self.concavity_zhand_free = param_dict['concavity_zhand_free']

        self.wrench_regularization_constant = param_dict['wrench_regularization_constant']


    def set_constraint_function_parameter_values(self, param_dict):

        # barrier function parameters for line/line plus point/line
        self.tr_friction_left_robot_pivot = param_dict['tr_friction_left_robot_pivot']
        self.tr_friction_right_robot_pivot = param_dict['tr_friction_right_robot_pivot']
        self.tr_torque_left_robot_pivot = param_dict['tr_torque_left_robot_pivot']
        self.tr_torque_right_robot_pivot = param_dict['tr_torque_right_robot_pivot']
        self.tr_max_normal_robot_pivot = param_dict['tr_friction_robot_left_pivot']
        self.tr_friction_left_external_pivot = param_dict['tr_friction_left_external_pivot']
        self.tr_friction_right_external_pivot = param_dict['tr_friction_right_external_pivot']
        self.tr_min_normal_external_pivot = param_dict['tr_min_normal_external_pivot']
        self.torque_margin_robot_pivot = param_dict['torque_margin_robot_pivot']

        # barrier function parameters for line/line plus point/line
        self.tr_friction_left_robot_free = param_dict['tr_friction_left_robot_free']
        self.tr_friction_right_robot_free = param_dict['tr_friction_right_robot_free']
        self.tr_torque_left_robot_free = param_dict['tr_torque_left_robot_free']
        self.tr_torque_right_robot_free = param_dict['tr_torque_right_robot_free']
        self.tr_max_normal_robot_free = param_dict['tr_friction_robot_left_free']

        self.friction_margin_robot_free = param_dict['friction_margin_robot_free']
        self.torque_margin_robot_free = param_dict['torque_margin_robot_free']
        self.l_contact_multiplier_robot_free = param_dict['l_contact_multiplier_robot_free']
  
        
