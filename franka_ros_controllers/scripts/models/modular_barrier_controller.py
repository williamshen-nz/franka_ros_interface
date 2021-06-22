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
    def __init__(self, param_dict):

        # create pbal kinematics helper
        self.pbal_helper = PbalHelper(param_dict = param_dict['obj_params'])

        # set objective function parameters
        self.set_objective_function_paramater_values(param_dict = param_dict)

        # set constraint function parameters
        self.set_constraint_function_parameter_values(param_dict = param_dict)

        # set maximum normal force robot can apply
        self.Nmax = param_dict['Nmax']

        # mode cost and constraint list
        self.mode_cost, self.mode_constraint = None, None


    def solve_for_delta_wrench(self):

        self.update_mode_cost_and_constraints_list()
        P, q = self.build_quadratic_program_cost()
        Aiq, biq = self.build_quadratic_program_constraints()
        return self.solve_quadratic_program(P, q, Aiq, biq)

    def build_quadratic_program_cost(self):

        P,q = np.zeros([3,3]), np.zeros(3)
        for cost in self.mode_cost:
            Pi, qi = cost()
            P += Pi
            q += qi

        return P, q

    def build_quadratic_program_constraints(self):

        Aiq, biq, trust_region, slacks = [], [], [], []

        for constraint in self.mode_constraint:
            aiqi, biqi, tri = constraint()
            Aiq.append(aiqi)
            biq.append(biqi)
            trust_region.append(tri)
            slacks.append(np.dot(aiqi, self.contact_wrench) - biqi)

        return np.array(Aiq), -np.array(trust_region) * np.array(
            slacks)

    def solve_quadratic_program(self, P, q, Aiq, biq):
        
        P_cvx = matrix(2 * P)
        q_cvx = matrix(q)
        
        Aiq_cvx = matrix(Aiq)
        biq_cvx = matrix(biq)

        result = solvers.qp(P_cvx, q_cvx, Aiq_cvx, biq_cvx)
        return np.squeeze(np.array(result['x']))

    def update_mode_cost_and_constraints_list(self):

        if self.mode == -1:  # line/line stick and point/line stick
            
            self.mode_cost = [
                self.theta_error_pivot_cost,
                self.wrench_regularization_robot_pivot_cost,
                self.normal_force_robot_pivot_cost
            ]

            self.mode_constraint = [
                self.friction_right_robot_pivot_constraint,
                self.friction_left_robot_pivot_constraint,
                self.torque_right_robot_pivot_constraint,
                self.torque_left_robot_pivot_constraint,
                self.normal_force_max_robot_pivot_constraint,
                self.friction_right_external_pivot_constraint,
                self.friction_left_external_pivot_constraint
            ]

        if self.mode == 0:   # line/line slide +  and point/line stick

            self.mode_cost = [
                self.theta_error_pivot_cost,
                self.slide_right_robot_pivot_cost,
                self.wrench_regularization_robot_pivot_cost
            ]

            self.mode_constraint = [
                self.friction_left_robot_pivot_constraint,
                self.torque_right_robot_pivot_constraint,
                self.torque_left_robot_pivot_constraint,
                self.normal_force_max_robot_pivot_constraint,
                self.friction_right_external_pivot_constraint,
                self.friction_left_external_pivot_constraint
            ]
         

        if self.mode == 1:   # line/line slide -  and point/line stick

            self.mode_cost = [
                self.theta_error_pivot_cost,
                self.slide_left_robot_pivot_cost,
                self.wrench_regularization_robot_pivot_cost
            ]

            self.mode_constraint = [
                self.friction_right_robot_pivot_constraint,
                self.torque_right_robot_pivot_constraint,
                self.torque_left_robot_pivot_constraint,
                self.normal_force_max_robot_pivot_constraint,
                self.friction_right_external_pivot_constraint,
                self.friction_left_external_pivot_constraint
            ]

 
        if self.mode == 2:   # line/line stick  and point/line slide +

            self.mode_cost = [
                self.theta_error_pivot_cost,
                self.slide_left_external_pivot_cost,
                self.wrench_regularization_robot_pivot_cost
            ]

            self.mode_constraint = [
                self.friction_right_robot_pivot_constraint,
                self.friction_left_robot_pivot_constraint,
                self.torque_right_robot_pivot_constraint,
                self.torque_left_robot_pivot_constraint,
                self.normal_force_max_robot_pivot_constraint,
                self.friction_right_external_pivot_constraint,
                self.normal_force_min_external_pivot_constraint
            ]

      
        if self.mode == 3:   # line/line stick  and point/line slide -

            self.mode_cost = [
                self.theta_error_pivot_cost,
                self.slide_right_external_pivot_cost,
                self.wrench_regularization_robot_pivot_cost
            ]

            self.mode_constraint = [
                self.friction_right_robot_pivot_constraint,
                self.friction_left_robot_pivot_constraint,
                self.torque_right_robot_pivot_constraint,
                self.torque_left_robot_pivot_constraint,
                self.normal_force_max_robot_pivot_constraint,
                self.friction_left_external_pivot_constraint,
                self.normal_force_min_external_pivot_constraint
            ]

        if self.mode == 4:   # free move
            pass

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
        if mode == 0 and err_s_pivot < 0:
            mode = -1
        if mode == 1 and err_s_pivot > 0:
            mode = -1
        if mode == 2 and err_x_pivot < 0:
            mode = -1
        if mode == 3 and err_x_pivot > 0:
            mode = -1
        
    	self.mode = mode

    	# update contact wrench
    	self.contact_wrench = contact_wrench


        if mode == -1:  # line/line stick and point/line stick
            self.err_theta_pivot = (2./np.pi) * np.arctan(
                err_theta_pivot/self.theta_scale_pivot)
            self.err_N_pivot = (2./np.pi) * np.arctan(
                (0.5 * self.Nmax - self.contact_wrench[0])/self.N_scale_pivot)

        if mode == 0:	# line/line slide +  and point/line stick
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_s_pivot = (2./np.pi) * np.arctan(
    			err_s_pivot/self.s_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
                (0.5 * self.Nmax - self.contact_wrench[0])/self.N_scale_pivot)

    	if mode == 1:	# line/line slide -  and point/line stick
    		self.err_theta_pivot = (2./np.pi) * np.arctan(
    			err_theta_pivot/self.theta_scale_pivot)
    		self.err_s_pivot = (2./np.pi) * np.arctan(
    			err_s_pivot/self.s_scale_pivot)
    		self.err_N_pivot = (2./np.pi) * np.arctan(
                (0.5 * self.Nmax - self.contact_wrench[0])/self.N_scale_pivot)

        if mode == 2: 	# line/line stick  and point/line slide +
            self.err_theta_pivot = (2./np.pi) * np.arctan(
                err_theta_pivot/self.theta_scale_pivot)
            self.err_x_pivot = (2./np.pi) * np.arctan(
                err_x_pivot/self.x_scale_pivot)
            self.err_N_pivot = (2./np.pi) * np.arctan(
                (0.5 * self.Nmax - self.contact_wrench[0])/self.N_scale_pivot)

        if mode == 3:	# line/line stick  and point/line slide -
            self.err_theta_pivot = (2./np.pi) * np.arctan(
                err_theta_pivot/self.theta_scale_pivot)
            self.err_x_pivot = (2./np.pi) * np.arctan(
                err_x_pivot/self.x_scale_pivot)
            self.err_N_pivot = (2./np.pi) * np.arctan(
                (0.5 * self.Nmax - self.contact_wrench[0])/self.N_scale_pivot)

    	if mode == 4:	# free move
    		self.err_theta_free = (2./np.pi) * np.arctan(
    			err_theta_free/self.theta_scale_free)
    		self.err_xhand_free = (2./np.pi) * np.arctan(
    			err_xhand_free/self.xhand_scale_free)
    		self.err_zhand_free = (2./np.pi) * np.arctan(
    			err_zhand_free/self.zhand_scale_free)      

    def friction_right_robot_pivot_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
        mu_c = self.pbal_helper.mu_contact
        aiq = np.array([-mu_c, 1., 0.])/np.sqrt(1 + mu_c ** 2)
        biq = 0.
        return aiq, biq, self.tr_friction_right_robot_pivot

    def friction_left_robot_pivot_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''
        mu_c = self.pbal_helper.mu_contact
        aiq = np.array([-mu_c, -1., 0.])/np.sqrt(1 + mu_c ** 2)
        biq = 0.
        return aiq, biq, self.tr_friction_left_robot_pivot

    def torque_right_robot_pivot_constraint(self):
        ''' right (i.e., positive) boundary of torque cone '''
        lc = self.pbal_helper.l_contact
        aiq = np.array([-lc / 2., 0., 1.])
        biq = -self.torque_margin_robot_pivot
        return aiq, biq, self.tr_torque_right_robot_pivot

    def torque_left_robot_pivot_constraint(self):
        ''' right (i.e., positive) boundary of torque cone '''
        lc = self.pbal_helper.l_contact
        aiq = np.array([-lc / 2., 0., -1.])
        biq = -self.torque_margin_robot_pivot
        return aiq, biq, self.tr_friction_left_robot_pivot

    def normal_force_max_robot_pivot_constraint(self):
        ''' maximum applied normal force constraint '''
        Nm = self.Nmax
        aiq = np.array([1., 0., 0.])
        biq = Nm
        return aiq, biq, self.tr_max_normal_robot_pivot

    def friction_right_external_pivot_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
        mu_g = self.pbal_helper.mu_ground
        R2C = self.pbal_helper.contact2robot(
            np.array([0., 0., self.theta_hand]))
        aiq = np.dot(np.array([-1., mu_g, 0.]), R2C)
        biq = 0.
        return aiq, biq, self.tr_friction_right_external_pivot

    def friction_left_external_pivot_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''
        mu_g = self.pbal_helper.mu_ground
        R2C = self.pbal_helper.contact2robot(np.array(
            [0., 0., self.theta_hand]))
        aiq = np.dot(np.array([1., mu_g, 0.]), R2C)
        biq = 0.
        return aiq, biq, self.tr_friction_left_external_pivot

    def normal_force_min_external_pivot_constraint(self):
        ''' normal force at external contact must be positive '''
        R2C = self.pbal_helper.contact2robot(
            np.array([0., 0., self.theta_hand]))
        aiq = np.dot(np.array([0., 1., 0.]), R2C)
        biq = 0.
        return aiq, biq, self.tr_min_normal_external_pivot

    def theta_error_pivot_cost(self):
        ''' cost term for rotating about pivot '''
        if self.s_hand is None or self.l_hand is None:
            proj_vec = self.concavity_theta_pivot * np.array(
                [0., 0., 1])
        else:
            proj_vec = self.concavity_theta_pivot * np.array(
                [-self.s_hand, self.l_hand, 1.])
        error = self.K_theta_pivot * self.err_theta_pivot
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def slide_right_robot_pivot_cost(self):
        ''' cost term for sliding right at robot '''
        proj_vec = self.concavity_s_pivot * np.array(
                [-self.pbal_helper.mu_contact, 1., 0.])
        error = self.K_s_pivot * self.err_s_pivot
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def slide_left_robot_pivot_cost(self):
        ''' cost term for sliding left at robot '''
        proj_vec = self.concavity_s_pivot * np.array(
            [-self.pbal_helper.mu_contact, -1., 0.])
        error = -self.K_s_pivot * self.err_s_pivot
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def slide_right_external_pivot_cost(self):
        ''' cost term for sliding right at external '''
        R2C = self.pbal_helper.contact2robot(np.array(
            [0., 0., self.theta_hand]))
        proj_vec = self.concavity_x_pivot * np.dot(np.array(
            [-1., -self.pbal_helper.mu_ground, 0.]), R2C)
        error = -self.K_x_pivot * self.err_x_pivot
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def slide_left_external_pivot_cost(self):
        ''' cost term for sliding left at external '''
        R2C = self.pbal_helper.contact2robot(np.array(
            [0., 0., self.theta_hand]))
        proj_vec = self.concavity_x_pivot * np.dot(np.array(
            [1., -self.pbal_helper.mu_ground, 0.]), R2C)
        error = self.K_x_pivot * self.err_x_pivot
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def normal_force_robot_pivot_cost(self):
        ''' cost encouraging normal force to be at Nmax/2 '''
        proj_vec = self.concavity_N_pivot * np.array(
            [1., 0., 0.])
        error = self.K_N_pivot * self.err_N_pivot
        print(error)
        return np.outer(proj_vec, proj_vec), -2 * proj_vec * error

    def wrench_regularization_robot_pivot_cost(self):
        ''' cost encouraging delta wrench to be small in 2-norm '''
        return self.wrench_regularization_constant * np.identity(
            3), np.zeros(3)

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
        self.theta_scale_pivot = param_dict['theta_scale_pivot']
        self.concavity_theta_pivot = param_dict['concavity_theta_pivot']
    
    	# objective function parameters: s, line/line plus point/line
        self.K_s_pivot = param_dict['K_s_pivot']
        self.s_scale_pivot = param_dict['s_scale_pivot']
        self.concavity_s_pivot = param_dict['concavity_s_pivot']
    
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
        self.theta_scale_free = param_dict['theta_scale_free']
        self.concavity_theta_free = param_dict['concavity_theta_free']

        # objective function parameters: x-hand, free
        self.K_xhand_free = param_dict['K_xhand_free']
        self.xhand_scale_free = param_dict['xhand_scale_free']
        self.concavity_xhand_free = param_dict['concavity_xhand_free']

        # objective function parameters: z-hand, free
        self.K_zhand_free = param_dict['K_zhand_free']
        self.zhand_scale_free = param_dict['zhand_scale_free']
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
  
        
