import pdb
import numpy as np
import scipy as scp
import matplotlib.pyplot as plt


class FrictionCone(object):

    def __init__(self, params):
        self.mass = params['mass']
        self.mu_left = params['mu_left']
        self.mu_right = params['mu_right']
        self.Nmax = params['Nmax']

    def cone_vertices(self):
        return [0, -self.mass], [-self.Nmax*self.mu_left, self.Nmax - self.mass
                                 ], [self.Nmax*self.mu_right, self.Nmax - self.mass]

    def plot_friction_cone(self, ax):

        v1, v2, v3 = self.cone_vertices()
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], color='y')
        ax.plot([v1[0], v3[0]], [v1[1], v3[1]], color='y')
        return ax

    def quasirandom_sample(self):

        tht_left = -np.arctan(self.mu_left)
        tht_right = np.arctan(self.mu_right)

        tht = np.random.uniform(low=tht_left, high=tht_right)
        N = np.random.uniform(low=0, high=self.Nmax)

        return N - self.mass, N*np.tan(tht)


def update_friction_parameters(left_boundary, right_boundary):

    # build LP
    list_size_left = left_boundary.shape[0]
    list_size_right = right_boundary.shape[0]
    RHS = np.concatenate(
        [left_boundary[:, 1], right_boundary[:, 1]])  # normal forces
    LHS = np.vstack([np.vstack([-left_boundary[:, 0], np.zeros(list_size_left), np.ones(list_size_left)]).T,
                     np.vstack([np.zeros(list_size_right), right_boundary[:, 0], np.ones(list_size_right)]).T])

    # solve LP
    params, _, _, _ = np.linalg.lstsq(LHS, RHS, rcond=1)
    mu_right, mu_left, mass = 1/params[0], 1/params[1], params[2]
    return mu_right, mu_left, mass


if __name__ == "__main__":

    params = {
        'mass': 0.3,
        'mu_left': 0.8,
        'mu_right': 0.4,
        'Nmax': 1.
    }

    fc = FrictionCone(params)

    # hyperparameters
    decay_rate = 0.995

    # initial conditions
    mu_left_0 = 0.2
    mu_right_0 = 0.2

    # initialize boundary list
    N = np.linspace(0.1, fc.Nmax, 3)
    fplot = np.linspace(-fc.Nmax, fc.Nmax)
    left_boundary = np.vstack([-mu_left_0 * N, N]).T
    right_boundary = np.vstack([mu_right_0 * N, N]).T

    # initialize parameters
    mu_left, mu_right, mass = update_friction_parameters(
        left_boundary, right_boundary)
    # left_boundary_angles = np.arctan2(left_boundary[:,1], left_boundary[:, 0])
    # right_boundary_angles = np.arctan2(right_boundary[:,1], right_boundary[:, 0])

    fig, ax = plt.subplots(1, 1)

    ax.scatter(left_boundary[:, 0], left_boundary[:, 1], color='r')
    ax.scatter(right_boundary[:, 0], right_boundary[:, 1], color='g')
    ax.plot(fplot, -1/mu_left * fplot - mass, 'r')
    ax.plot(fplot, 1/mu_right * fplot - mass, 'g')

    fn_list = []
    ft_list = []

    for i in range(1000):

        ax.clear()

        fn, ft = fc.quasirandom_sample()

        # angle = np.arctan2(ft, fn)
        is_outside_left = fn + 1/mu_left * ft < - 0.7 * mass 
        is_outside_right = fn - 1/mu_right * ft < -0.7 * mass

        if is_outside_left:
            # index = np.argmin(np.abs(fn - left_boundary[:, 1]))
            left_boundary = np.append(left_boundary, 
                np.expand_dims(np.array([ft, fn]), axis=0), axis=0)

        elif is_outside_right:
            # index = np.argmin(np.abs(fn - right_boundary[:, 1]))
            right_boundary = np.append(right_boundary, 
                np.expand_dims(np.array([ft, fn]), axis=0), axis=0)
        else:
            ft_list.append(ft)
            fn_list.append(fn)

        mu_left, mu_right, mass = update_friction_parameters(
            left_boundary, right_boundary)

        # # if i % 10 == 0:
        left_boundary[:, 0] *= decay_rate
        right_boundary[:, 0] *= decay_rate


        # plot
        ax = fc.plot_friction_cone(ax)
        ax.scatter(ft_list, fn_list, color='b')
        ax.plot(fplot, -1/mu_left * fplot + mass, 'r')
        ax.plot(fplot, 1/mu_right * fplot + mass, 'g')
        ax.axvline(x=0, color='k')
        ax.scatter(left_boundary[:, 0], left_boundary[:, 1], color='r')
        ax.scatter(right_boundary[:, 0], right_boundary[:, 1], color='g')
        ax.set_xlim([-fc.Nmax, fc.Nmax])
        ax.set_ylim([-fc.Nmax, fc.Nmax])

        plt.pause(0.1)
        print(mass)
        # pdb.set_trace()

    plt.show()
