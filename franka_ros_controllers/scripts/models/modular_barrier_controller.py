import copy
import json
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
        self.param_dict = param_dict

        # mode cost and constraint list
        self.mode_cost, self.mode_constraint = None, None


    def solve_for_delta_wrench(self):
        P, q, proj_vec_list, error_list = \
            self.build_quadratic_program_cost()
        Aiq, biq, slacks = self.build_quadratic_program_constraints()
        delta_wrench =  self.solve_quadratic_program(P, q, Aiq, slacks)
        delta_wrench_unconstrained = np.linalg.solve(2*P, -q) 
        debug_str = self.build_debug_string(delta_wrench, delta_wrench_unconstrained, 
            proj_vec_list, error_list, Aiq, biq, slacks)
        return delta_wrench, debug_str


    def build_debug_string(self, delta_wrench, delta_wrench_unconstrained,
        proj_vec_list, error_list, Aiq, biq, slacks):

        debug_dict = {
            "mode" : self.mode,
            "delta_wrench" : delta_wrench.tolist(),
            "delta_wrench_unconstrained" : delta_wrench_unconstrained.tolist(),
            "proj_vec_list" : [proj_veci.tolist() for proj_veci in proj_vec_list],
            "error_list": error_list,
            "constraint_normals": Aiq.tolist(),
            "constraint_offsets" : biq.tolist(),
            "slacks" : slacks.tolist(),
            "measured_wrench" : self.contact_wrench.tolist()
        }

        debug_str = json.dumps(debug_dict)
        return debug_str


    def build_quadratic_program_cost(self):

        P,q = np.zeros([3,3]), np.zeros(3)
        proj_vec_list, error_list = [], []
        for cost in self.mode_cost:
            Pi, qi, proj_veci, errori = cost()
            P += Pi
            q += qi
            proj_vec_list.append(proj_veci)
            error_list.append(errori)

        return P, q, proj_vec_list, error_list

    def build_quadratic_program_constraints(self):

        Aiq, biq, trust_region, slacks, slack_product = [], [], [], [], []

        for constraint in self.mode_constraint:
            aiqi, biqi, tri = constraint()
            Aiq.append(aiqi)
            biq.append(biqi)
            trust_region.append(tri)
            slacki = np.dot(aiqi, self.contact_wrench) - biqi

            if slacki<0:
                slack_producti = -tri[0] * slacki
            else:
                slack_producti = -tri[1] * slacki


            slacks.append(slacki)
            slack_product.append(slack_producti)

        return np.array(Aiq), np.array(biq
            ), np.array(slack_product)

    def solve_quadratic_program(self, P, q, Aiq, biq):
        
        P_cvx = matrix(2 * P)
        q_cvx = matrix(q)
        
        Aiq_cvx = matrix(Aiq)
        biq_cvx = matrix(biq)

        result = solvers.qp(P_cvx, q_cvx, Aiq_cvx, biq_cvx)
        return np.squeeze(np.array(result['x']))

    def update_controller(self, mode, theta_hand, contact_wrench, err_dict,
    	l_hand = None,
    	s_hand = None):

    	self.err_dict=err_dict

    	# update pose variables
    	self.theta_hand, self.l_hand, self.s_hand = theta_hand, l_hand, s_hand

    	# update_mode
        if mode == 0 and err_dict["err_s"] < 0:
            mode = -1
        if mode == 1 and err_dict["err_s"] > 0:
            mode = -1
        if mode == 2 and err_dict["err_x_pivot"] < 0:
            mode = -1
        if mode == 3 and err_dict["err_x_pivot"] > 0:
            mode = -1
        
    	self.mode = mode

    	# update contact wrench
    	self.contact_wrench = contact_wrench

        self.R2C = self.pbal_helper.contact2robot(np.array([
            self.l_hand, self.s_hand, self.theta_hand]))

        if self.mode == -1:  # line/line stick and point/line stick
            self.current_params=self.param_dict['pivot_params']

            self.mode_cost = [
                self.theta_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]

        if self.mode == 0:   # line/line slide +  and point/line stick
            self.current_params=self.param_dict['pivot_params']

            self.mode_cost = [
                self.theta_cost,
                self.slide_right_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]
         

        if self.mode == 1:   # line/line slide -  and point/line stick
            self.current_params=self.param_dict['pivot_params']

            self.mode_cost = [
                self.theta_cost,
                self.slide_left_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]

 
        if self.mode == 2:   # line/line stick  and point/line slide +
            self.current_params=self.param_dict['pivot_params']

            self.mode_cost = [
                self.theta_cost,
                self.slide_left_external_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.normal_force_min_external_constraint
            ]

      
        if self.mode == 3:   # line/line stick  and point/line slide -
            self.current_params=self.param_dict['pivot_params']

            self.mode_cost = [
                self.theta_cost,
                self.slide_right_external_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_left_external_constraint,
                self.normal_force_min_external_constraint
            ]

        if self.mode == 4:   # guarded move
            self.current_params=self.param_dict['guarded_move_params']

            self.mode_cost = [
                self.xhand_cost,
                self.zhand_cost,
                self.theta_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.normal_force_min_contact_constraint
            ]

        if self.mode == 5:   #  static object flush move
            self.current_params=self.param_dict['static_object_flush_move_params']

            self.mode_cost = [
                self.xhand_cost,
                self.zhand_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.normal_force_min_contact_constraint
            ]

        if self.mode == 6:

            self.current_params=self.param_dict['pure_stick_params']

            self.mode_cost = [
                self.wrench_regularization_cost,
                self.normal_force_robot_pivot_cost
                ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint
                ]

    def compute_error_theta(self):
        self.err_theta = self.compute_general_error(
            error_value=self.err_dict['err_theta'],
            scale_value=self.current_params['theta_scale'])

    def compute_error_N(self):
        self.err_N = self.compute_general_error(
            error_value=(self.current_params['N_tar'] - self.contact_wrench[0]),
            scale_value=self.current_params['N_scale'])

    def compute_error_s(self):
        self.err_s = self.compute_general_error(
            error_value=self.err_dict['err_s'],
            scale_value=self.current_params['s_scale'])

    def compute_error_x_pivot(self):    
        self.err_x_pivot = self.compute_general_error(
            error_value=self.err_dict['err_x_pivot'],
            scale_value=self.current_params['x_pivot_scale'])

    def compute_error_xhand(self):
        self.err_xhand = self.compute_general_error(
            error_value=self.err_dict['err_xhand'],
            scale_value=self.current_params['hand_pos_scale'])

    def compute_error_zhand(self):
        self.err_zhand = self.compute_general_error(
            error_value=self.err_dict['err_zhand'],
            scale_value=self.current_params['hand_pos_scale'])


    def compute_general_error(self, error_value, scale_value):
        return (2./np.pi)* np.arctan(error_value/scale_value)

    def slide_right_robot_cost(self):
        ''' cost term for sliding right at robot '''
        self.compute_error_s()
        if self.current_params['use_measured_mu_contact'] and (self.pbal_helper.mu_contact is not None):
            mu_c = self.pbal_helper.mu_contact
        else:
            mu_c = self.current_params['mu_contact']

        return self.general_cost(
            base_error=self.err_s,
            base_vec=np.array([-mu_c, 1., 0.]),
            K=self.current_params['K_s'],
            concavity=self.current_params['concavity_s'])

    def slide_left_robot_cost(self):
        ''' cost term for sliding left at robot '''
        self.compute_error_s()
        if self.current_params['use_measured_mu_contact'] and (self.pbal_helper.mu_contact is not None):
            mu_c = self.pbal_helper.mu_contact
        else:
            mu_c = self.current_params['mu_contact']

        return self.general_cost(
            base_error=-self.err_s,
            base_vec=np.array([-mu_c, -1., 0.]),
            K=self.current_params['K_s'],
            concavity=self.current_params['concavity_s'])

    def slide_right_external_cost(self):
        ''' cost term for sliding right at external '''
        self.compute_error_x_pivot()
        if self.current_params['use_measured_mu_ground'] and (self.pbal_helper.mu_ground is not None):
            mu_g = self.pbal_helper.mu_ground
        else:
            mu_g = self.current_params['mu_ground']

        return self.general_cost(
            base_error=-self.err_x_pivot,
            base_vec=np.dot(np.array([-1., -mu_g, 0.]), self.R2C),
            K=self.current_params['K_x_pivot'],
            concavity=self.current_params['concavity_x_pivot'])

    def slide_left_external_cost(self):
        ''' cost term for sliding left at external '''
        self.compute_error_x_pivot()
        if self.current_params['use_measured_mu_ground'] and (self.pbal_helper.mu_ground is not None):
            mu_g = self.pbal_helper.mu_ground
        else:
            mu_g = self.current_params['mu_ground']

        return self.general_cost(
            base_error=self.err_x_pivot,
            base_vec=np.dot(np.array([1., -mu_g, 0.]), self.R2C),
            K=self.current_params['K_x_pivot'],
            concavity=self.current_params['concavity_x_pivot'])

    def normal_force_cost(self):
        ''' cost encouraging normal force to be at N_tar '''
        self.compute_error_N()
        return self.general_cost(
            base_error=self.err_N,
            base_vec=np.array([1., 0, 0.]),
            K=self.current_params['K_N'],
            concavity=self.current_params['concavity_N'])

    def wrench_regularization_cost(self):
        ''' cost encouraging delta wrench to be small in 2-norm '''
        return self.current_params['wrench_regularization_constant'] * np.identity(
            3), np.zeros(3), np.zeros(3), 0.

    def xhand_cost(self):
        ''' cost minimizing x-error in free move in world frame '''
        self.compute_error_xhand()               
        return self.general_cost(
            base_error=self.err_xhand,
            base_vec= np.dot(np.array([1., 0., 0.]), self.R2C),
            K=self.current_params['K_pos_hand'],
            concavity=self.current_params['concavity_pos_hand'])

    def zhand_cost(self):
        ''' cost minimizing z-error in free move in world frame '''
        self.compute_error_zhand()
        return self.general_cost(
            base_error=self.err_zhand,
            base_vec= np.dot(np.array([0., 1., 0.]), self.R2C),
            K=self.current_params['K_pos_hand'],
            concavity=self.current_params['concavity_pos_hand'])

    def theta_cost(self):
        ''' cost term for rotating about pivot '''
        self.compute_error_theta()
        if self.s_hand is None or self.l_hand is None:
            if self.mode == -1:
                base_vec = np.array([0., -.01, 1.])
            else: 
                base_vec = np.array([0., 0., 1.])
        else:
            base_vec = np.array([-self.s_hand, self.l_hand, 1.])
        return self.general_cost(
            base_error=self.err_theta,
            base_vec=base_vec,
            K=self.current_params['K_theta'],
            concavity=self.current_params['concavity_theta'])

    def general_cost(self, base_error, base_vec, K, concavity):
        proj_vec = base_vec*concavity
        error = base_error*K
        return np.outer(proj_vec, proj_vec
            ), -2 * proj_vec * error, proj_vec, error

    def friction_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
        if self.current_params['use_measured_mu_contact'] and (self.pbal_helper.mu_contact is not None):
            mu_c = self.pbal_helper.mu_contact
        else:
            mu_c = self.current_params['mu_contact']

        aiq = np.array([-mu_c, 1., 0.])/np.sqrt(1 + mu_c ** 2)
        biq = -self.current_params['friction_margin']
        return aiq, biq, self.current_params['tr_friction']

    def friction_left_contact_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''
        if self.current_params['use_measured_mu_contact'] and (self.pbal_helper.mu_contact is not None):
            mu_c = self.pbal_helper.mu_contact
        else:
            mu_c = self.current_params['mu_contact']

        aiq = np.array([-mu_c, -1., 0.])/np.sqrt(1 + mu_c ** 2)
        biq = -self.current_params['friction_margin']
        return aiq, biq, self.current_params['tr_friction']

    def torque_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of torque cone '''
        lc = self.pbal_helper.l_contact * self.current_params['l_contact_multiplier']
        aiq = np.array([-lc / 2., 0., 1.])
        biq = -self.current_params['torque_margin']
        return aiq, biq, self.current_params['tr_torque']

    def torque_left_contact_constraint(self):
        ''' left (i.e., negative) boundary of torque cone '''
        lc = self.pbal_helper.l_contact * self.current_params['l_contact_multiplier']
        aiq = np.array([-lc / 2., 0., -1.])
        biq = -self.current_params['torque_margin']
        return aiq, biq, self.current_params['tr_torque']

    def normal_force_max_contact_constraint(self):
        ''' maximum applied normal force at contact constraint '''
        Nm = self.current_params['Nmax_contact']
        aiq = np.array([1., 0., 0.])
        biq = Nm
        return aiq, biq, self.current_params['tr_max_normal_contact']

    def normal_force_min_contact_constraint(self):
        ''' maximum applied normal force at contact constraint '''
        Nm = self.current_params['Nmin_contact']
        aiq = np.array([-1., 0., 0.])
        biq = -Nm
        return aiq, biq, self.current_params['tr_min_normal_contact']

    def normal_force_min_external_constraint(self):
        ''' normal force at external contact must be above 0 '''
        aiq = np.dot(np.array([0., 1., 0.]), self.R2C)
        biq = 0.
        return aiq, biq, self.current_params['tr_min_normal_external']

    def friction_right_external_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
        if self.current_params['use_measured_mu_ground'] and (self.pbal_helper.mu_ground is not None):
            mu_g = self.pbal_helper.mu_ground
        else:
            mu_g = self.current_params['mu_ground']
        aiq = np.dot(np.array([-1., mu_g, 0.]), self.R2C)
        biq = 0.
        return aiq, biq, self.current_params['tr_friction_external']

    def friction_left_external_constraint(self):
        ''' left (i.e., positive) boundary of friction cone '''
        if self.current_params['use_measured_mu_ground'] and (self.pbal_helper.mu_ground is not None):
            mu_g = self.pbal_helper.mu_ground
        else:
            mu_g = self.current_params['mu_ground']

        aiq = np.dot(np.array([1., mu_g, 0.]), self.R2C)
        biq = 0.
        return aiq, biq, self.current_params['tr_friction_external']
