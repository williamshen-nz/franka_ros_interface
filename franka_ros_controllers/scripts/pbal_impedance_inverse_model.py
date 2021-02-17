import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
import pdb

from pbal_helper import PbalHelper


class PbalImpedanceInverseModel(object):
    def __init__(self, param_dict):

        # object parameters
        self.pbal_helper = PbalHelper(param_dict['obj_params'])

        # pose target parameters
        self.contact_pose_target = param_dict['contact_pose_target']

    def static_equilbrium_contact(self):

        d, s, theta = self.contact_pose_target[0], self.contact_pose_target[
            1], self.contact_pose_target[2]
        beq = -self.pbal_helper.mgl * np.sin(theta - self.pbal_helper.theta0)
        Aeq = np.array([-s, d, 1.])

        return np.expand_dims(Aeq, axis=0), np.array([beq])

    def static_equilbrium_robot(self):

        Aeq_contact, beq_contact = self.static_equilbrium_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)

        return np.dot(Aeq_contact, robot2contact), beq_contact

    def static_equilbrium_robot_aux(self):
       
        Aeq_robot, beq_robot = self.static_equilbrium_robot()

        return np.hstack([Aeq_robot, np.array([[0]])]), beq_robot

    def wrench_cone_constraints_line_contact(self):

        mu, lc = self.pbal_helper.mu_contact, self.pbal_helper.l_contact

        # written in the form Aiq x <= biq
        biq = np.zeros(4)
        Aiq = np.array([[-mu, 1., 0.], [-mu, -1., 0.], [-lc / 2., 0., 1.],
                        [-lc / 2., 0., -1.]])

        return Aiq, biq

    def wrench_cone_constraints_line_robot(self):

        Aiq_contact, biq_contact = self.wrench_cone_constraints_line_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)
        return np.dot(Aiq_contact, robot2contact), biq_contact
    
    def wrench_cone_constraints_line_robot_aux(self):
        
        Aiq_contact, biq_contact = self.wrench_cone_constraints_line_contact()
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)
        
        temp1 = np.dot(Aiq_contact, robot2contact)
        temp2 = np.dot(Aiq_contact, np.array([-1, 0, 0]))

        return np.hstack([temp1, np.expand_dims(temp2, axis=1)]), biq_contact

    def wrench_cone_constraint_pivot_robot(self):

        mu = self.pbal_helper.mu_ground
        biq = np.zeros(2)
        Aiq = np.array([[1., mu, 0.], [-1., mu, 0]])
        return Aiq, biq

    def wrench_cone_constraint_pivot_robot_aux(self):

        Aiq, biq = self.wrench_cone_constraint_pivot_robot()

        temp2 = np.expand_dims(np.array([-1., 0., 0.]), axis=1)
        return np.hstack([Aiq, np.dot(Aiq, temp2)]), biq

    def normal_force_constraint_contact(self, Nmax):

        Aiq_contact = np.array([1., 0., 0.])
        biq_contact = Nmax
        return np.expand_dims(Aiq_contact, axis=0), np.array([biq_contact])

    def normal_force_constraint_robot(self, Nmax):

        Aiq_contact, biq_contact = self.normal_force_constraint_contact(Nmax)
        robot2contact = self.pbal_helper.contact2robot(
            self.contact_pose_target)
        return np.dot(Aiq_contact, robot2contact), biq_contact
    
    def normal_force_constraint_robot_aux(self, Nmax):

        Aiq_robot, biq_robot = self.normal_force_constraint_robot(Nmax)
        return np.hstack([Aiq_robot, np.array([[0]])]), biq_robot

    def solve_linear_program(self, Nmax):

        Aeq_r, beq_r = self.static_equilbrium_robot()

        Aline_r, bline_r = self.wrench_cone_constraints_line_robot()
        Apivot_r, bpivot_r = self.wrench_cone_constraint_pivot_robot()
        Anormal_r, bnormal_r = self.normal_force_constraint_robot(Nmax)

        Aiq = np.vstack([Aline_r, Apivot_r, Anormal_r])
        biq = np.concatenate([bline_r, bpivot_r, bnormal_r])

        result2 = self.solve_lp_cvxopt(np.array([10., 0., 0.]), Aeq_r, beq_r,
                                       Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))

        return robot_wrench
    
    def solve_linear_program_aux(self, Nmax):

        Aeq_aux, beq_aux = self.static_equilbrium_robot_aux()

        Aline_aux, bline_aux = self.wrench_cone_constraints_line_robot_aux()
        Apivot_aux, bpivot_aux = self.wrench_cone_constraint_pivot_robot_aux()
        Anormal_aux, bnormal_aux = self.normal_force_constraint_robot_aux(Nmax)

        # positive aux constraints
        Aiq_aux = np.expand_dims(np.array([0., 0., 0., - 1]), axis=0)
        biq_aux = np.array([0.])

        Aiq = np.vstack([Aline_aux, Apivot_aux, Anormal_aux, Aiq_aux])
        biq = np.concatenate([bline_aux, bpivot_aux, bnormal_aux, biq_aux])

        beta = -1. 
        cost = np.array([0., 0., 0., beta])

        result2 = self.solve_lp_cvxopt(cost, Aeq_aux, beq_aux,
                                       Aiq, biq)

        robot_wrench = np.squeeze(np.array(result2['x']))

        return robot_wrench

    def solve_lp_cvxopt(self, c, Aeq, beq, Aiq, biq):

        c_cvxopt = matrix(c)

        Aeq_cvxopt = matrix(Aeq)
        beq_cvxopt = matrix(beq)

        Aiq_cvxopt = matrix(Aiq)
        biq_cvxopt = matrix(biq)

        result = solvers.lp(c_cvxopt, Aiq_cvxopt, biq_cvxopt, Aeq_cvxopt,
                            beq_cvxopt)
        return result

    def plot_state(self, robot_wrench, ax, Nmax):

        AXIS_LENGTH = 0.02
        FORCE_SCALE = 0.002
        TORQUE_SCALE = 0.05

        contact_pose = self.contact_pose_target
        robot_pose = self.pbal_helper.forward_kin(contact_pose)
        contact2robot = self.pbal_helper.contact2robot(robot_pose)

        # robot x-axis
        ax.plot(self.pbal_helper.pivot[0] + np.array([0, AXIS_LENGTH]),
                self.pbal_helper.pivot[1] + np.array([0, 0]),
                'r',
                linewidth=3)
        # robot z-axis
        ax.plot(self.pbal_helper.pivot[0] + np.array([0, 0]),
                self.pbal_helper.pivot[1] + np.array([0, AXIS_LENGTH]),
                'b',
                linewidth=3)
        # plot pivot
        ax.plot(self.pbal_helper.pivot[0],
                self.pbal_helper.pivot[1],
                'k.',
                markersize=10)

        # contact x-axis
        ax.plot(
            robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[0, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH * contact2robot[0, 1]]),
            'r',
            linewidth=3)
        # contact y-axis
        ax.plot(
            robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[1, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH * contact2robot[1, 1]]),
            'g',
            linewidth=3)
        # contact point
        ax.plot(robot_pose[0], robot_pose[1], 'k.', markersize=10)

        # pivot to project point
        projection_point = self.pbal_helper.pivot + contact_pose[
            0] * contact2robot[:2, 0]
        ax.plot(np.array([self.pbal_helper.pivot[0], projection_point[0]]),
                np.array([self.pbal_helper.pivot[1], projection_point[1]]),
                'k',
                linewidth=1)
        # projection to contact
        contact_point = projection_point + contact_pose[1] * contact2robot[:2,
                                                                           1]
        ax.plot(np.array([projection_point[0], contact_point[0]]),
                np.array([projection_point[1], contact_point[1]]),
                'k',
                linewidth=1)

        # torque
        if robot_wrench[2] > 0:
            tcolor = 'y'
        else:
            tcolor = 'b'
        cc = plt.Circle((robot_pose[0], robot_pose[1]),
                        TORQUE_SCALE * np.abs(robot_wrench[2]),
                        color=tcolor)
        cmax = plt.Circle((robot_pose[0], robot_pose[1]),
                        TORQUE_SCALE * Nmax * self.pbal_helper.l_contact/2,
                        edgecolor='k',
                        facecolor='w')
        ax.add_artist(cmax)
        ax.add_artist(cc)

        # generators in contact frame
        fp_contact = np.array([Nmax, Nmax * self.pbal_helper.mu_contact, 0])
        fm_contact = np.array([Nmax, -Nmax * self.pbal_helper.mu_contact, 0])

        # generators in world frame
        fp_world = np.dot(contact2robot, fp_contact)
        fm_world = np.dot(contact2robot, fm_contact)

        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, fp_world[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, fp_world[1]]),
                'k--',
                linewidth=2)
        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, fm_world[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, fm_world[1]]),
                'k--',
                linewidth=2)

        # force
        ax.plot(robot_pose[0] + FORCE_SCALE * np.array([0, robot_wrench[0]]),
                robot_pose[1] + FORCE_SCALE * np.array([0, robot_wrench[1]]),
                'k',
                linewidth=2)

        ax.plot(self.pbal_helper.pivot[0] +
                FORCE_SCALE * np.array([0, Nmax * self.pbal_helper.mu_ground]),
                self.pbal_helper.pivot[1] + FORCE_SCALE * np.array([0, Nmax]),
                'k--',
                linewidth=2)
        ax.plot(
            self.pbal_helper.pivot[0] +
            FORCE_SCALE * np.array([0, -Nmax * self.pbal_helper.mu_ground]),
            self.pbal_helper.pivot[1] + FORCE_SCALE * np.array([0, Nmax]),
            'k--',
            linewidth=2)
        # pivot_force
        ax.plot(self.pbal_helper.pivot[0] +
                FORCE_SCALE * np.array([0, -robot_wrench[0]]),
                self.pbal_helper.pivot[1] +
                FORCE_SCALE * np.array([0, -robot_wrench[1]]),
                'k',
                linewidth=2)


if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .6
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.15
    obj_params['mu_ground'] = 1.0
    obj_params['l_contact'] = 0.065

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = np.array([-0.1, 0.02, np.pi/6])

    # find force
    Nmax = 20
    pbal_impedance_inv = PbalImpedanceInverseModel(param_dict)
    robot_wrench = pbal_impedance_inv.solve_linear_program_aux(Nmax)
    print(robot_wrench)

    # plot
    fig, ax = plt.subplots(1, 1)
    ax.invert_xaxis()
    pbal_impedance_inv.plot_state(robot_wrench, ax, Nmax)
    ax.axis('equal')
    plt.show()
    