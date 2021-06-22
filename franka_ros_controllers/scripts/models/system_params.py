class SystemParams(object):

    def __init__(self):

        self.object_params = {
            "L_CONTACT_MAX": 0.1,                     # m (length of robot/object contact)
            "MU_GROUND_0": 0.75,                       # friction between obj/ground
            "MU_CONTACT_0": 0.1,                     # friciton between robot/obj
            "TORQUE_BOUNDARY_MARGIN": 0.8             # multiplies L_CONTACT to set conservative margin for torque boundary
        }

        self.estimator_params = {
            "RATE": 100.,                           # hz
            "NBATCH_PIVOT": 250,                    # max number of datapoints for estimation for pivot 
            "UPDATE_LENGTH_PIVOT": 200,             # number of good points before update/publish for pivot
            "ANGLE_DIFF_THRESH_PIVOT": 0.005,       # rad(difference in angles for new datapoints for pivot)
            "SLIDING_THRESH_FRICTION_EST": 0.03,    # m/s (threshold for sliding velocity)
            "NBATCH_GRAV_PARAMS": 500,              # number of good points before update/publish for grav
            "ANGLE_DIFF_THRESH_GRAV": 0.001,        # rad (difference in angles for new datapoints for grav)
            "DELTA_ANGLE_THRESH_GRAV": 0.25,        # rad (minimum travel needed to publish grav params)
            "NORMAL_FORCE_THRESHOLD_FORCE": 0.05    # N (min normal force to register as being in contact)
        }

        self.controller_params = {
            "IMPEDANCE_STIFFNESS_LIST": [1000, 1000, 1000, 100, 30, 100],
            "TORQUE_UPPER": [40, 40, 36, 36, 32, 28, 24],                  # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            "FORCE_UPPER": [100, 100, 100, 25, 25, 25],                    # default [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
            "RATE": 30,                                                    # hz (control rate)
            "INTEGRAL_MULTIPLIER": 5.,
            "Nmax": 30,                                                    # N (maximum normal force robot can apply)
            "K_theta_pivot": 150.,                                         # objective function parameters: theta, line/line plus point/line
            "theta_scale_pivot": 0.3,
            "concavity_theta_pivot": 60,
            "K_s_pivot": 1.,                                               # objective function parameters: s, line/line plus point/line
            "s_scale_pivot": 0.0005,
            "concavity_s_pivot": 0.3,
            "K_x_pivot": 108.,                                             # objective function parameters: s, line/line plus point/line
            "x_scale_pivot": 0.006,
            "concavity_x_pivot": 60,
            "K_N_pivot": 5.,                                               # objective function parameters: N, line/line plus point/line
            "N_scale_pivot": 8.,
            "concavity_N_pivot": 1.,
            "K_theta_free": 1.,                                            # objective function parameters: theta, free
            "theta_scale_free": 1.5,
            "concavity_theta_free": 1.,    
            "K_xhand_free": 1.,                                            # objective function parameters: x-hand, free
            "xhand_scale_free": 1.5,
            "concavity_xhand_free": 1.,
            "K_zhand_free": 1.,                                            # objective function parameters: z-hand, free
            "zhand_scale_free": 1.5,
            "concavity_zhand_free": 1.,
            "wrench_regularization_constant": 0.00001,
            "tr_friction_left_robot_pivot": 1.,                            # barrier function parameters for line/line plus point/line
            "tr_friction_right_robot_pivot": 1.,
            "tr_torque_left_robot_pivot": 3,
            "tr_torque_right_robot_pivot": 3,
            "tr_friction_robot_left_pivot": 1,
            "tr_friction_left_external_pivot": 1,
            "tr_friction_right_external_pivot": 1,
            "tr_min_normal_external_pivot": 1,
            "torque_margin_robot_pivot_factor": 0.002,                   
            "tr_friction_left_robot_free": 1.,                              # barrier function parameters for line/line plus point/line
            "tr_friction_right_robot_free": 1.,
            "tr_torque_left_robot_free": 1.,
            "tr_torque_right_robot_free": 1.,
            "tr_friction_robot_left_free": 1., 
            "friction_margin_robot_free": 1.,
            "torque_margin_robot_free": 1.,
            "l_contact_multiplier_robot_free": 1.
        }
