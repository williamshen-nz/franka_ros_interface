import numpy as np
import matplotlib.pyplot as plt
import pdb

class PbalPhysics(object):
    def __init__(self, param_dict):

        # object parameters
        self.pivot = param_dict['pivot']
        self.mgl = param_dict['mgl']
        self.theta0 = param_dict['theta0']
        self.mu_contact = param_dict['mu_contact']
        self.l_contact = param_dict['l_contact']

        # impedance parameters
        self.impedance_target = param_dict['impedance_target']
        self.impedance_stiffness = param_dict['impedance_stiffness']

    def contact2robot(self, contact_pose):
        
        # unpack
        theta = contact_pose[2]

        # define sine and cosine
        sint, cost = np.sin(theta), np.cos(theta)

        # line contact orientation in world frame
        return np.array([[-sint, -cost, 0], 
            [-cost, sint, 0], [0., 0., 1.]])

    def forward_kin(self, contact_pose):

        # rotation from contact to robot
        contact2robot = self.contact2robot(contact_pose)

        # equivalent coordinates in robot frame
        robot_pose = np.append(self.pivot, 0.
            ) + np.dot(contact2robot, contact_pose)

        return robot_pose

    def inverse_kin(self, robot_pose):

        # rotation from robot to contact
        robot2contact = self.contact2robot(robot_pose).T

        # equivalent coordinates in robot frame
        robot_pose = np.dot(robot2contact, robot_pose - np.append(
            self.pivot, 0.))

        return robot_pose

    def impedance_model_robot(self, robot_pose):

        # target minus current
        delta_pose = self.impedance_target - robot_pose

        return -self.impedance_stiffness * delta_pose
        
    def impedance_model_contact(self, contact_pose):

        # robot pose
        robot_pose = self.forward_kin(contact_pose)
    
        # wrench in robot frame
        impedance_wrench_robot = self.impedance_model_robot(robot_pose)

        # rotation from robot to contact
        robot2contact = self.contact2robot(contact_pose).T

        # wrench in contact frame
        return np.dot(robot2contact, impedance_wrench_robot) 

    def torque_balance(self, contact_pose, contact_wrench):

        d, s, theta = contact_pose[0], contact_pose[1], contact_pose[2]
        Fd, Fs, tau = contact_wrench[0], contact_wrench[1], contact_wrench[2]

        return (
            self.mgl*np.sin(theta - self.theta0)
            + Fs * d
            - Fd * s
            + tau        
        )

    def equilibrium_check(self, contact_pose):

        # contact wrench based on controller
        contact_wrench = self.impedance_model_contact(contact_pose)

        # net torque
        return self.torque_balance(contact_pose, contact_wrench)

    def find_equilibrium_angle(self, d, s, theta_guess):

        # params
        TOL, DTHETA = 1e-6, 1e-4

        # pack
        guess_pose = np.array([d, s, theta_guess])

        # check if at equilbrium
        net_torque = self.equilibrium_check(guess_pose)

        while np.abs(net_torque) > TOL:

            # gradient
            ntp = self.equilibrium_check(guess_pose + np.array([0, 0, DTHETA]))
            ntm = self.equilibrium_check(guess_pose + np.array([0, 0, -DTHETA]))
            dnt_dtheta = (ntp - ntm) / (2 * DTHETA)
            
            # current value
            net_torque = self.equilibrium_check(guess_pose)

            # new guess
            theta_guess = theta_guess - net_torque / dnt_dtheta
            print(theta_guess)
            guess_pose[-1] = theta_guess

        return theta_guess

    def plot_state(self, robot_pose, ax):

        AXIS_LENGTH = 0.2

        # robot x-axis
        ax.plot(self.pivot[0] + np.array([0, AXIS_LENGTH]),
            self.pivot[1] + np.array([0, 0]), 'r' ,linewidth=3)
        # robot z-axis
        ax.plot(self.pivot[0] + np.array([0, 0]),
            self.pivot[1] + np.array([0, AXIS_LENGTH]), 'b',linewidth=3)
        # plot pivot
        ax.plot(self.pivot[0], self.pivot[1], 'k.', markersize=10)

        contact2robot = self.contact2robot(robot_pose)

        # contact x-axis
        ax.plot(robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[0, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH*contact2robot[0, 1]]), 'r' ,linewidth=3)
        # contact y-axis
        ax.plot(robot_pose[0] + np.array([0, AXIS_LENGTH * contact2robot[1, 0]]),
            robot_pose[1] + np.array([0, AXIS_LENGTH*contact2robot[1, 1]]), 'g' ,linewidth=3)
        # contact point
        ax.plot(robot_pose[0], robot_pose[1], 'k.', markersize=10)

        # plot in contact axes
        contact_pose = self.inverse_kin(robot_pose)
        # pivot to project point
        projection_point = self.pivot + contact_pose[0] * contact2robot[:2, 0]
        ax.plot(np.array([self.pivot[0], projection_point[0]]),
            np.array([self.pivot[1], projection_point[1]]), 'k', linewidth=1)
        # projection to contact
        contact_point = projection_point + contact_pose[1]*contact2robot[:2, 1]
        ax.plot(np.array([projection_point[0], contact_point[0]]),
            np.array([projection_point[1], contact_point[1]]), 'k', linewidth=1)
            
        robotpose_2 = self.forward_kin(contact_pose)
        print(robot_pose - robotpose_2)

            
        

if __name__ == "__main__":

    param_dict = dict()

    # object parameters
    param_dict['pivot'] = np.array([0.,0.])
    param_dict['mgl'] = 0.
    param_dict['theta0'] = 0.
    param_dict['mu_contact'] = 0.15
    param_dict['l_contact'] = 0.1

    # impedance parameters
    param_dict['impedance_target'] = np.array([0.0, 0.9, 0])
    param_dict['impedance_stiffness'] = np.array([1e3, 0.2e3, 30])

    # create obj
    pbal_obj = PbalPhysics(param_dict)
    contact_pose = np.array([-1, 0, 0.1])

    theta_eq = pbal_obj.find_equilibrium_angle(*contact_pose)

    fig, ax = plt.subplots(1,1)
    ax.invert_xaxis()    
    pbal_obj.plot_state(pbal_obj.forward_kin(contact_pose), ax)
    ax.axis('equal')
    plt.show()
    


