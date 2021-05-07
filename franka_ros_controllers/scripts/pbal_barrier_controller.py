import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
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

        return np.dot(Aiq, measured_wrench) - biq, Aiq

    def barrier_function_values(self, slacks):
        ''' evaluates barrier function f = exp(k*slacks) '''

        k = self.exponential_time_constant
        return -k*slacks

    def qp_cost_values(self, contact_pose, delta_contact_pose, mode):
        ''' computes cost function for the QP '''

        # unpack
        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]
        delta_s, delta_theta = delta_contact_pose[1], delta_contact_pose[2]

        # theta error cost
        a1 = np.array([-s, d, 1.])
        b1 = self.K_theta * delta_theta
        P = np.outer(a1, a1)
        q = -2 * a1 * b1
            
        if mode == -1:   # sticking, sticking
            pass
        elif mode == 0:    # sticking pivot, robot slide right

            # s error cost slide right
            a2 = self.K_s * delta_s
            b2 = np.array([self.pbal_helper.mu_contact, 1., 0.])
            P  = P + np.outer(a2, a2)
            q  = q - 2 * a2 * b2

        elif mode == 1:    # sticking pivot, robot slide left

            # s error slide left
            a2 = self.K_s * delta_s
            b2 = np.array([self.pbal_helper.mu_contact, -1., 0.])
            P  = P + np.outer(a2, a2)
            q  = q - 2 * a2 * b2
        else:
            raise RuntimeError("Invalid mode: must be -1, 0, or 1")

        return P, q
        

    def solve_qp(self, measured_wrench, contact_pose, 
        delta_contact_pose, mode, Nmax):
        '''
        mode -1: sticking, sticking
        mode 0: sticking pivot, robot slide right 
        mode 1: sticking pivot, robot slide left
        mode not in [0, 1]: 
        ''' 

        # compute cost
        P, q = self.qp_cost_values(contact_pose, 
            delta_contact_pose, mode)

        # compute slacks
        slacks = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        # compute iq constraint slacks and normals
        slacks, normals = self.compute_slack_values(contact_pose, 
            measured_wrench, Nmax)

        # compute valube of barrier functi
        fbarrier = self.barrier_function_values(slacks)

        # select constraints
        if mode == -1: # sticking, sticking

            # inequality constraints
            Aiq = normals
            biq = fbarrier

        elif mode == 0: # sticking pivot, robot slide right 
            
            # inequality constraints
            Aiq = normals[2:6, :]
            biq = fbarrier[2:6]

        elif mode == 1: # sticking pivot, robot slide left 
            
            # inequality constraints
            Aiq = normals[2:6, :]
            biq = fbarrier[2:6]

        else:
            raise RuntimeError("Invalid mode: must be -1, 0, or 1")   


        result = self.solve_qp_cvxopt(P, q, Aiq, biq)
        return np.squeeze(np.array(result['x']))


    def solve_qp_cvxopt(self, P, q, Aiq, biq):

        P_cvxopt = matrix(2 * P)
        q_cvxopt = matrix(q)
        
        Aiq_cvxopt = matrix(Aiq)
        biq_cvxopt = matrix(biq)

        result = solvers.qp(P_cvxopt, q_cvxopt,
            Aiq_cvxopt, biq_cvxopt)
        return result


if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.3
    obj_params['mu_ground'] = 0.75
    obj_params['l_contact'] = 0.065

    # position control parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['K_theta'] = 1.
    param_dict['K_s'] = 1.
    param_dict['exponential_time_constant'] = 1.

    pbc = PbalBarrierController(param_dict)

    measured_wrench = np.array([10., 1., .1])
    contact_pose = np.array([0.1, 0.1, 0.1])
    delta_contact_pose = np.array([0, 0.01, 0.2])
    mode = -1

    result = pbc.solve_qp(measured_wrench, contact_pose, 
        delta_contact_pose, mode, 20.)



    pdb.set_trace()
