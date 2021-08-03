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

from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
            end_effector_wrench)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    base_wrench = data
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
            base_wrench)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def enumerate_vertices_of_constraint_polygon(theta_list,b_list,closed=False:)
    num_constraints = len(theta_list)
    A = np.zeros([num_constraints,2])
    B = np.array(b_list)

    vertex_x_list = []
    vertex_y_list = []

    #build constraint matrix for the polygon
    for i in range(num_external_params):
        A[i][0] = np.cos(theta_list[i])
        A[i][1] = np.sin(theta_list[i])

    theta_start = (theta_list[0]+theta_list[0])/2
    cost_start = np.array([-np.cos(theta_start),-np.sin(theta_start)])

    sol = solvers.lp(matrix(cost_start),matrix(A),matrix(B))

    #find the first vertex
    x_sol = np.squeeze(sol['x'])
    vertex_x_list.append(x_sol[0])
    vertex_y_list.append(x_sol[1])

    #active constraints for the first vertex
    active_constraints = np.squeeze(sol['z'])>1e-5

    #find the active constraint for the first vertex is the closest to theta_start in angle, and is CW of theta_start
    if active_constraints[0]:
        index_CW=0
    else:
        index_CW = num_constraints-1
        while active_constraints[index_CW]==False:
            index_CW = index_CW-1

    #find the active constraint for the first vertex that is tclosest to theta_start in angle, and is CCW of theta_start
    index_CCW = 1
    while active_constraints[index_CCW]==False:
        index_CCW = index_CCW+1









if __name__ == '__main__':
    measured_contact_wrench_list = []
    measured_base_wrench_list = []

    rospy.init_node("wrench_cone_estimation")
    rospy.sleep(1.0)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    num_divisions = 8
    theta_max = np.pi/2
    theta_range = theta_max*(1.0*np.array(range(-num_divisions,num_divisions+1)))/num_divisions + np.pi/2


    theta_range = np.hstack([np.array([-np.pi/2]),theta_range,np.array([-np.pi/2])])

    num_external_params = len(theta_range)

    A_external_stats_intermediate = np.zeros([num_external_params,3])
    B_external_stats_intermediate = np.zeros(num_external_params)

    vertex_solve_mat = np.zeros([num_external_params,2])


    offset_list = np.zeros([num_external_params])
    stats_external_list = []

    boundary_update_time = 1
    last_update_time = time.time()

    distance_threshold = .5
    distance_threshold2 = .5

    for i in range(num_external_params):
       stats_external_list.append(livestats.LiveStats([.99]))
       A_external_stats_intermediate[i][0] = np.cos(theta_range[i])
       A_external_stats_intermediate[i][1] = np.sin(theta_range[i])
       vertex_solve_mat[i][0] = np.cos(theta_range[i])
       vertex_solve_mat[i][1] = np.sin(theta_range[i])

    redundancy_check_matrix = matrix(vertex_solve_mat)
    
    redundancy_check_cost_list = []
    for i in range(num_external_params):
        redundancy_check_cost_list.append(matrix(-vertex_solve_mat[i]))



    fig, axs = plt.subplots(1,3)

    external_plot_list = []
    external_plot_list_right = []
    external_plot_list_left = []
    for i in range(num_external_params):
        external_line_plot, = axs[2].plot([0], [0], 'b')
        external_line_plot_right, = axs[2].plot([0], [0], 'r')
        external_line_plot_left, = axs[2].plot([0], [0], 'b')
        external_plot_list.append(external_line_plot)
        external_plot_list_right.append(external_line_plot_right)
        external_plot_list_left.append(external_line_plot_left)


    external_boundary_plot, = axs[2].plot([0], [0], 'k')


    while not rospy.is_shutdown():

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)


        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                axs[2].plot(measured_base_wrench[0], measured_base_wrench[1], 'r*')
                b_temp = np.dot(A_external_stats_intermediate,measured_base_wrench)
                for i in range(num_external_params):
                    stats_external_list[i].add(b_temp[i])
            for i in range(num_external_params):
                B_external_stats_intermediate[i] = stats_external_list[i].quantiles()[0][1]


        if time.time()- last_update_time> boundary_update_time:
            last_update_time = time.time() 

            B_redundancy_check = matrix(B_external_stats_intermediate)
            non_redundant_indices = []
            for i in range(num_external_params):
                sol = solvers.lp(redundancy_check_cost_list[i],redundancy_check_matrix,B_redundancy_check)
                #print sol
                #print np.squeeze(sol['z'])
                #if B_external_stats_intermediate[i] - np.dot(vertex_solve_mat[i],np.squeeze(sol['x'])) < 1e-6:
                #print  -np.dot(vertex_solve_mat[i],np.squeeze(sol['x']))
                #print  sol['primal objective']
                if B_external_stats_intermediate[i] + sol['primal objective'] < 1e-6:
                    
                    non_redundant_indices.append(i)

            vertex_x_list = []
            vertex_y_list = []
            vertex_x1_list = []
            vertex_y1_list = []
            vertex_x2_list = []
            vertex_y2_list = []


            for i in range(len(non_redundant_indices)-1):
                vertex_solve_mat_temp = vertex_solve_mat[[non_redundant_indices[i],non_redundant_indices[i+1]]]
                vertex_solve_b_temp = B_external_stats_intermediate[[non_redundant_indices[i],non_redundant_indices[i+1]]]
                vertex_temp = np.linalg.solve(vertex_solve_mat_temp,vertex_solve_b_temp)
                vertex_x_list.append(vertex_temp[0])
                vertex_y_list.append(vertex_temp[1])

            vertex_x_list= [vertex_x_list[0]]+vertex_x_list+[vertex_x_list[-1]]
            vertex_y_list= [vertex_y_list[0]]+vertex_y_list+[vertex_y_list[-1]]

            for i in range(len(vertex_x_list)):
                vertex_temp=[vertex_x_list[i],vertex_y_list[i]]
                if np.mod(i,2)==0:
                    if len(vertex_x1_list)==0 or np.sqrt((vertex_x1_list[-1]-vertex_temp[0])**2
                        +(vertex_y1_list[-1]-vertex_temp[1])**2) > distance_threshold:
                        vertex_x1_list.append(vertex_temp[0])
                        vertex_y1_list.append(vertex_temp[1])
                else:
                    if len(vertex_x2_list)==0 or np.sqrt((vertex_x2_list[-1]-vertex_temp[0])**2
                        +(vertex_y2_list[-1]-vertex_temp[1])**2) > distance_threshold:
                        vertex_x2_list.append(vertex_temp[0])
                        vertex_y2_list.append(vertex_temp[1])

            theta_final_with_redundancies = []
            A_final_with_redundancies = np.zeros([len(vertex_x1_list)+len(vertex_x2_list)-2,2])
            B_final_with_redundancies = np.zeros(len(vertex_x1_list)+len(vertex_x2_list)-2)

            count1= 0
            count2= 0

            first_vertex = None
            last_vertex = None 
            while count1<len(vertex_x1_list)-1 or count2<len(vertex_x2_list)-1:
                used_list1 = None
                if count1 == len(vertex_x1_list)-1 and count2 < len(vertex_x2_list)-1:
                    theta2 = np.arctan2(vertex_y2_list[count2+1]-vertex_y2_list[count2],vertex_x2_list[count2+1]-vertex_x2_list[count2])-np.pi/2
                    theta_final_with_redundancies.append(theta2)
                    A_final_with_redundancies[count1+count2][0]=np.cos(theta2)
                    A_final_with_redundancies[count1+count2][1]=np.sin(theta2)
                    B_final_with_redundancies[count1+count2]=np.cos(theta2)*vertex_x2_list[count2]+np.sin(theta2)*vertex_y2_list[count2]
                    count2=count2+1
                    used_list1 = False

                if count1 < len(vertex_x1_list)-1 and count2 == len(vertex_x2_list)-1:
                    theta1 = np.arctan2(vertex_y1_list[count1+1]-vertex_y1_list[count1],vertex_x1_list[count1+1]-vertex_x1_list[count1])-np.pi/2
                    theta_final_with_redundancies.append(theta1)
                    A_final_with_redundancies[count1+count2][0]=np.cos(theta1)
                    A_final_with_redundancies[count1+count2][1]=np.sin(theta1)
                    B_final_with_redundancies[count1+count2]=np.cos(theta1)*vertex_x1_list[count1]+np.sin(theta1)*vertex_y1_list[count1]
                    count1=count1+1
                    used_list1 = True 

                if count1 < len(vertex_x1_list)-1 and count2 < len(vertex_x2_list)-1:
                    theta1 = np.arctan2(vertex_y1_list[count1+1]-vertex_y1_list[count1],vertex_x1_list[count1+1]-vertex_x1_list[count1])-np.pi/2
                    theta2 = np.arctan2(vertex_y2_list[count2+1]-vertex_y2_list[count2],vertex_x2_list[count2+1]-vertex_x2_list[count2])-np.pi/2

                    while theta1>3*np.pi/2:
                        theta1=theta1-2*np.pi
                    while theta1<-np.pi/2:
                        theta1=theta1+2*np.pi
                    while theta2>3*np.pi/2:
                        theta2=theta2-2*np.pi
                    while theta2<-np.pi/2:
                        theta2=theta2+2*np.pi

                    if theta1<theta2:
                        theta_final_with_redundancies.append(theta1)
                        A_final_with_redundancies[count1+count2][0]=np.cos(theta1)
                        A_final_with_redundancies[count1+count2][1]=np.sin(theta1)
                        B_final_with_redundancies[count1+count2]=np.cos(theta1)*vertex_x1_list[count1]+np.sin(theta1)*vertex_y1_list[count1]
                        count1=count1+1
                        used_list1 = True 

                    else:
                        theta_final_with_redundancies.append(theta2)
                        A_final_with_redundancies[count1+count2][0]=np.cos(theta2)
                        A_final_with_redundancies[count1+count2][1]=np.sin(theta2)
                        B_final_with_redundancies[count1+count2]=np.cos(theta2)*vertex_x2_list[count2]+np.sin(theta2)*vertex_y2_list[count2]
                        count2=count2+1
                        used_list1 = False 
                if count1+count2 == 1:
                    if used_list1:
                        first_vertex = [vertex_x1_list[0],vertex_y1_list[0]]
                    else:
                        first_vertex = [vertex_x2_list[0],vertex_y2_list[0]]
                if count1+count2 == len(vertex_x1_list)+len(vertex_x2_list)-2:
                    if used_list1:
                        last_vertex = [vertex_x1_list[-1],vertex_y1_list[-1]]
                    else:
                        last_vertex = [vertex_x2_list[-1],vertex_y2_list[-1]]

            non_redundant_indices = []
            if len(B_final_with_redundancies)>0:
                B_redundancy_check = matrix(B_final_with_redundancies)
                second_redundancy_check_matrix = matrix(A_final_with_redundancies)
                non_redundant_indices = []
                for i in range(len(B_final_with_redundancies)):
                    if i==0 or i == len(B_final_with_redundancies)-1:
                        non_redundant_indices.append(i)
                    else:
                        cost_matrix = matrix(-A_final_with_redundancies[i])
                        sol = solvers.lp(cost_matrix,second_redundancy_check_matrix,B_redundancy_check)
                        if B_final_with_redundancies[i] - np.dot(A_final_with_redundancies[i],np.squeeze(sol['x'])) < 1e-6:
                            non_redundant_indices.append(i)


            print time.time() - last_update_time

            A_final_no_redundancies = A_final_with_redundancies[non_redundant_indices]
            B_final_no_redundancies = B_final_with_redundancies[non_redundant_indices]

            if len(B_final_no_redundancies)>0:

                vertex_x_list_with_repeats = [first_vertex[0]]
                vertex_y_list_with_repeats = [first_vertex[1]]

                for i in range(len(B_final_no_redundancies)-1):
                    vertex_temp = np.linalg.solve(A_final_no_redundancies[[i,i+1]],B_final_no_redundancies[[i,i+1]])
                    vertex_x_list_with_repeats.append(vertex_temp[0])
                    vertex_y_list_with_repeats.append(vertex_temp[1])
                vertex_x_list_with_repeats.append(last_vertex[0])
                vertex_y_list_with_repeats.append(last_vertex[1])

                vertex_x_list = [vertex_x_list_with_repeats[0]]
                vertex_y_list = [vertex_y_list_with_repeats[0]]
                for i in range(0,len(vertex_x_list_with_repeats)):
                    if np.sqrt((vertex_x_list[-1]-vertex_x_list_with_repeats[i])**2+(vertex_y_list[-1]-vertex_y_list_with_repeats[i])**2)>distance_threshold2:
                        vertex_x_list.append(vertex_x_list_with_repeats[i])
                        vertex_y_list.append(vertex_y_list_with_repeats[i])

            
                A_final_no_redundancies = np.zeros([len(vertex_x_list)-1,3])
                B_final_no_redundancies = np.zeros(len(vertex_x_list)-1)

                A_external_right = np.zeros([0,3])
                B_external_right = np.zeros(0)

                A_external_left = np.zeros([0,3])
                B_external_left = np.zeros(0)
                for i in range(len(vertex_x_list)-1):
                    theta = np.arctan2(vertex_y_list[i+1]-vertex_y_list[i],vertex_x_list[i+1]-vertex_x_list[i])-np.pi/2
                    while theta>3*np.pi/2:
                        theta=theta-2*np.pi
                    while theta<-np.pi/2:
                        theta=theta+2*np.pi

                    row_to_append = np.array([np.cos(theta),np.sin(theta),0])
                    val_to_append = np.array([np.cos(theta)*vertex_x_list[i]+np.sin(theta)*vertex_y_list[i]])

                    if theta>np.pi/2:
                        A_external_right = np.vstack([A_external_right,row_to_append])
                        B_external_right = np.hstack([B_external_right,val_to_append])
                    else:
                        A_external_left = np.vstack([A_external_left,row_to_append])
                        B_external_left = np.hstack([B_external_left,val_to_append])

                    A_final_no_redundancies[i][0]=np.cos(theta)
                    A_final_no_redundancies[i][1]=np.sin(theta)
                    B_final_no_redundancies[i]=np.cos(theta)*vertex_x_list[i]+np.sin(theta)*vertex_y_list[i]



                external_boundary_plot.set_xdata(vertex_x_list)
                external_boundary_plot.set_ydata(vertex_y_list)
                print len(B_final_no_redundancies)



            # for i in range(num_external_params):
            #     if i<len(B_final_no_redundancies):
            #         P0 =[A_final_no_redundancies[i][0]*B_final_no_redundancies[i],A_final_no_redundancies[i][1]*B_final_no_redundancies[i]]
            #         external_plot_list[i].set_xdata([P0[0]-100*A_final_no_redundancies[i][1],P0[0]+100*A_final_no_redundancies[i][1]])
            #         external_plot_list[i].set_ydata([P0[1]+100*A_final_no_redundancies[i][0],P0[1]-100*A_final_no_redundancies[i][0]])
            #     else:
            #         external_plot_list[i].set_xdata(0)
            #         external_plot_list[i].set_ydata(0)

                for i in range(num_external_params):
                    if i<len(B_external_right):
                        P0 =[A_external_right[i][0]*B_external_right[i],A_external_right[i][1]*B_external_right[i]]
                        external_plot_list_right[i].set_xdata([P0[0]-100*A_external_right[i][1],P0[0]+100*A_external_right[i][1]])
                        external_plot_list_right[i].set_ydata([P0[1]+100*A_external_right[i][0],P0[1]-100*A_external_right[i][0]])
                    else:
                        external_plot_list_right[i].set_xdata(0)
                        external_plot_list_right[i].set_ydata(0)

                for i in range(num_external_params):
                    if i<len(B_external_left):
                        P0 =[A_external_left[i][0]*B_external_left[i],A_external_left[i][1]*B_external_left[i]]
                        external_plot_list_left[i].set_xdata([P0[0]-100*A_external_left[i][1],P0[0]+100*A_external_left[i][1]])
                        external_plot_list_left[i].set_ydata([P0[1]+100*A_external_left[i][0],P0[1]-100*A_external_left[i][0]])
                    else:
                        external_plot_list_left[i].set_xdata(0)
                        external_plot_list_left[i].set_ydata(0)
              
    




            



        axs[0].set_xlim([-30, 30])
        axs[0].set_ylim([-10, 50])
        axs[0].set_title('Friction Cone Contact')

        axs[1].set_ylim([-10, 50])
        axs[1].set_xlim([-10, 10])
        axs[1].set_title('Torque Cone Contact')

        axs[2].set_xlim([-30, 30])
        axs[2].set_ylim([-50, 10])
        axs[2].set_title('Friction Cone Ground')
        # plt.show()
        plt.pause(0.3)




