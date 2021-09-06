#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import models.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from models.system_params import SystemParams
from convex_hull_estimator import ConvexHullEstimator


from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6




if __name__ == '__main__':

    num_divisions = 50
  
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions


    hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, distance_threshold=.5, closed = True)


    fig, axs = plt.subplots(2,2)
    fig2, axs2 = plt.subplots(1,1)

    polygon_radius = 25
    #num_polygon_vertices = np.random.randint(low=3,high=10)
    num_polygon_vertices = 3
    theta_range_polygon = 2*np.pi*(1.0*np.array(range(num_polygon_vertices)))/num_polygon_vertices

    polygon_radii = polygon_radius*np.random.rand(num_polygon_vertices)+1
    polygon_vertices = np.zeros([num_polygon_vertices,2])

    for i in range(num_polygon_vertices):
        polygon_vertices[i][0] = polygon_radii[i] * np.cos(theta_range_polygon[i])
        polygon_vertices[i][1] = polygon_radii[i] * np.sin(theta_range_polygon[i])

    polygon_hull = ConvexHull(polygon_vertices)
    hull_vertices = polygon_vertices[polygon_hull.vertices]

    num_hull_vertices = len(polygon_hull.vertices)




    offset_radius = 15
    offset_coord = offset_radius*(np.random.rand(2)-.5)

    for i in range(num_hull_vertices):
        hull_vertices[i][0]+=offset_coord[0]
        hull_vertices[i][1]+=offset_coord[1]
        
        #axs[1][1].plot(hull_vertices[i][0], hull_vertices[i][1], 'g.')

    max_x = np.max(np.transpose(hull_vertices)[0])
    min_x = np.min(np.transpose(hull_vertices)[0])
    delta_x =  max_x-min_x

    max_x+=delta_x/4
    min_x-=delta_x/4

    max_y = np.max(np.transpose(hull_vertices)[1])
    min_y = np.min(np.transpose(hull_vertices)[1])
    delta_y =  max_y-min_y

    max_y+=delta_y/4
    min_y-=delta_y/4

    num_data_points = 500
    noise_radius = 4
    for i in range(num_data_points):
        convex_combo = np.random.rand(num_hull_vertices)

        cumulative_sum = convex_combo[0]
        for j in range(1,num_hull_vertices-1):
            convex_combo[j]=(1-cumulative_sum)*convex_combo[j]
            cumulative_sum+=convex_combo[j]
        convex_combo[-1] = 1-cumulative_sum

        data_point = np.dot(np.random.permutation(convex_combo),hull_vertices)

        data_point = data_point - noise_radius*(np.random.rand(2)-.5)

        hull_estimator.add_data_point(data_point)

        axs[0][0].plot(data_point[0], data_point[1], 'r.')
        axs[0][1].plot(data_point[0], data_point[1], 'r.')
        axs[1][0].plot(data_point[0], data_point[1], 'r.')
        axs[1][1].plot(data_point[0], data_point[1], 'r.')

    last_update_time = time.time() 

    hull_estimator.generate_convex_hull_closed_polygon()

    print time.time() - last_update_time

    hull_estimator.initialize_quantile_boundary_plot(axs[0][0])
    hull_estimator.initialize_quantile_polygon_plot(axs[0][1])
    hull_estimator.initialize_polygon_star_plot(axs[1][0])
    
    
    

    hull_estimator.update_quantile_boundary_plot()
    hull_estimator.update_quantile_polygon_plot()
    hull_estimator.update_polygon_star_plot()

    hull_estimator.initialize_final_constraint_plot(axs[1][1])
    hull_estimator.update_final_constraint_plot()

    hull_estimator.initialize_side_detection_plot(axs2)
    hull_estimator.update_side_detection_plot()
    
    # hull_estimator.initialize_final_constraint_plot_left_right(axs[1][1])
    # hull_estimator.update_final_constraint_plot_left_right()

    axs2.set_xlim([0, 4*np.pi])
    axs2.set_ylim([-2, 20])

    axs[0][0].set_xlim([min_x, max_x])
    axs[0][0].set_ylim([min_y, max_y])
    axs[0][0].set_title('Quantile Boundaries')

    axs[0][1].set_xlim([min_x, max_x])
    axs[0][1].set_ylim([min_y, max_y])
    axs[0][1].set_title('Interior Polygon')

    axs[1][0].set_xlim([min_x, max_x])
    axs[1][0].set_ylim([min_y, max_y])
    axs[1][0].set_title('Star Operation')

    axs[1][1].set_xlim([min_x, max_x])
    axs[1][1].set_ylim([min_y, max_y])
    axs[1][1].set_title('Final Estimate')

    # axs[0][0].set_xlim([-50, 50])
    # axs[0][0].set_ylim([-50, 50])
    # axs[0][0].set_title('Quantile Boundaries')

    # axs[0][1].set_xlim([-50, 50])
    # axs[0][1].set_ylim([-50, 50])
    # axs[0][1].set_title('Interior Polygon')

    # axs[1][0].set_xlim([-50, 50])
    # axs[1][0].set_ylim([-50, 50])
    # axs[1][0].set_title('Star Operation')

    # axs[1][1].set_xlim([-50, 50])
    # axs[1][1].set_ylim([-50, 50])
    # axs[1][1].set_title('Final Estimate')
    plt.show()
