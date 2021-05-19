import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from pbal_impedance_inverse_model import PbalImpedanceInverseModel

if __name__ == "__main__":

    # object parameters
    obj_params = dict()
    obj_params['pivot'] = np.array([0., 0.])
    obj_params['mgl'] = .7
    obj_params['theta0'] = np.pi/12
    obj_params['mu_contact'] = 0.2
    obj_params['mu_ground'] = 1.0
    obj_params['l_contact'] = 0.065

    # position control parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = np.array([-0.1, 0.00, -np.pi / 6])

    # inverse model
    pbal_impedance_inv = PbalImpedanceInverseModel(param_dict)

    Nmax = 40  # maximum force
    dnom = -0.11  # nominal value of d

    NPTS = 20
    theta_vec, s_vec = np.linspace(-np.pi / 4, np.pi / 4,
                                   NPTS), np.linspace(-0.04, 0.04, NPTS)
    theta_grid, s_grid = np.meshgrid(theta_vec, s_vec)
    is_feasible = np.zeros([NPTS, NPTS, 3])

    for i in range(NPTS):
        for j in range(NPTS):
            for k in range(3):
                print(s_grid[i, j], (180/np.pi) * theta_grid[i,j])
                # update current pose
                current_pose = np.array([dnom, s_grid[i, j], theta_grid[i, j]])
                pbal_impedance_inv.contact_pose_target = current_pose

                # solve for wrench
                robot_wrench = pbal_impedance_inv.solve_linear_program_mode(Nmax,
                                                                            mode=k-1)
                
                if robot_wrench is not None:
                    is_feasible[i, j, k] = 1
    

    fig, axs = plt.subplots(1,3)
    titles = ['Sticking', 'Sliding Plus', 'Sliding Minus']
    for mode, ax in enumerate(axs):
        is_feasible_mode = is_feasible[:, :, mode]
        ax.scatter((180/np.pi) * theta_grid[is_feasible_mode == 1], s_grid[is_feasible_mode == 1], color='g')
        ax.scatter((180/np.pi) * theta_grid[is_feasible_mode == 0], s_grid[is_feasible_mode == 0], color='r')
        ax.set_xlabel('Theta [deg]')
        ax.set_ylabel('S [m]')
        ax.set_title(titles[mode])

    plt.show()
