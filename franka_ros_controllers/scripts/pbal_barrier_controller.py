import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_helper import PbalHelper


class PbalBarrierController(object):
    def __init__(self, param_dict):

        # object parameters
        self.pbal_helper = PbalHelper(param_dict['obj_params'])

        # position control parameters
        self.K_theta = param_dict['K_theta']
        self.K_s = param_dict['K_s']

        # barrier function parameters
        self.exponential_time_constant = param_dict[
            'exponential_time_constant']

    def compute_slack_values(self, contact_pose, 
        measured_wrench, Nmax):
        ''' computes slacks and and gradients '''

        # extract parameters
        mu_c, mu_g, lc = self.pbal_helper.mu_contact, \
            self.pbal_helper.mu_ground, self.pbal_helper.l_contact

        # compute pivot constraints
        contact2robot = self.pbal_helper.contact2robot(
            contact_pose)
        Aiq_pivot_robot = np.array([[1., mu_g, 0.], # friction 1
            [-1., mu_g, 0]]) # friction 2
        Aiq_pivot_contact = np.dot(Aiq_pivot_robot, contact2robot)

        # measured wrench is in contact frame
        biq = np.array([0., 0., 0., 0., Nmax, 0., 0.])
        Aiq = np.array([[-mu_c, 1., 0.], # friction robot 1
            [-mu_c, -1., 0.], # friction robot 2
            [-lc / 2., 0., 1.], # torque 1
            [-lc / 2., 0., -1.], # torque 2
            [1., 0., 0.]]) # normal force

        Aiq = np.vstack([Aiq, Aiq_pivot_contact]) # pivot friction 

        pdb.set_trace()
        return np.dot(Aiq, measured_wrench) - biq, Aiq

    def barrier_function_values(self, slacks):
        ''' evaluates barrier function f = exp(k*slacks) '''

        k = self.exponential_time_constant
        return np.exp(k*slacks)

    def qp_cost_values(self, contact_pose, contact_delta, mode):

        

    def solve_qp(self, slacks, measured_wrench, 
        contact_pose, delta_contact_pose, mode, Nmax):
        '''
        mode -1: sticking, sticking
        mode 0: sticking pivot, robot slide right 
        mode 1: sticking pivot, robot slide left
        mode not in [0, 1]: 
        ''' 
        slacks, normals = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        fbarrier = self.barrier_function_values(slacks)

        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]



        if mode == 0:
            
            # inequality constraints
            Aiq = fbarrier[2:6] * normals[2:6, :]
            biq = -slacks[2:6]
            
            s_control_direction = np.array([
                self.pbal_helper.mu_contact, 1, 0])
            theta_control_direction = np.array([-s/d, 1, 1/d]) 


        elif mode == 1:
            
            # inequality constraints
            Aiq = fbarrier[2:6] * normals[2:6, :]
            biq = -slacks[2:6]

            s_control_direction = np.array([
                -self.pbal_helper.mu_contact, 1, 0])
            theta_control_direction = np.array([-s/d, 1, 1/d])

        elif mode == -1:

            # inequality constraints
            Aiq = fbarrier * normals
            biq = -slacks

            theta_control_direction = np.array([-s/d, 1, 1/d])

        result = solve_qp_cvxopt(P, q, [], [], Aiq, biq)




    def solve_qp_cvxopt(self, Q, q, Aeq, beq, Aiq, biq):

        Q_cvxopt = matrix(Q)
        q_cvxopt = matrix(q)

        Aeq_cvxopt = matrix(Aeq)
        beq_cvxopt = matrix(beq)

        Aiq_cvxopt = matrix(Aiq)
        biq_cvxopt = matrix(biq)

        result = solvers.lp(Q_cvxopt, q_cvxopt, Aiq_cvxopt, biq_cvxopt, Aeq_cvxopt,
                            beq_cvxopt)
        return result










if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.3
    obj_params['mu_ground'] = 0.3
    obj_params['l_contact'] = 0.065

    # position control parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['K_theta'] = 1.
    param_dict['K_s'] = 1.

    pbc = PbalBarrierController(param_dict)

    measured_wrench = np.array([10., 1., .1])
    contact_pose = np.array([0.1, 0.1, 0.1])

    slacks, normals = pbc.compute_slack_values(
        contact_pose, measured_wrench, 20)

    pdb.set_trace()
