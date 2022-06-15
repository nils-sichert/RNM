#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt

class robot_kinematics:
    def __init__(self, debug=False):
        self.debug = debug

    def get_pose_from_angles(self, theta):
        ''' Get pose matrix A from joint angles. Utilizing pre-computed expressions using DH-parameters.
            
            Parameters:
                theta (List): Containing 7 joint angles

            Returns:
                A (Matrix): Pose matrix
                    Position is in A[9:12]
                    Rotation is in np.reshape(A[0:9], (3,3))
        '''
        
        sin1, sin2, sin3, sin4, sin5, sin6, sin7 = sin(theta[0]), sin(theta[1]), sin(theta[2]), sin(theta[3]), sin(theta[4]), sin(theta[5]), sin(theta[6])
        cos1, cos2, cos3, cos4, cos5, cos6, cos7 = cos(theta[0]), cos(theta[1]), cos(theta[2]), cos(theta[3]), cos(theta[4]), cos(theta[5]), cos(theta[6])
        

        # Calculate pose matrix A using pre-computed expression
        A = np.array([((((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*cos2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*sin2)*cos1 - (((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3)*sin1, 
        ((((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*cos2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*sin2)*sin1 + (((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3)*cos1, 
        -(((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*sin2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*cos2, 
        ((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*cos1 - (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 - (sin5*sin7*cos6 + cos5*cos7)*cos3)*sin1, 
        ((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*sin1 + (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 - (sin5*sin7*cos6 + cos5*cos7)*cos3)*cos1, 
        -(((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*sin2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*cos2, 
        (((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*cos2 + (sin4*sin6*cos5 - cos4*cos6)*sin2)*cos1 - ((sin4*cos6 + sin6*cos4*cos5)*sin3 + sin5*sin6*cos3)*sin1, 
        (((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*cos2 + (sin4*sin6*cos5 - cos4*cos6)*sin2)*sin1 + ((sin4*cos6 + sin6*cos4*cos5)*sin3 + sin5*sin6*cos3)*cos1, 
        -((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*sin2 + (sin4*sin6*cos5 - cos4*cos6)*cos2, 
        ((-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*cos2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*sin2)*cos1 - ((0.207*sin6 + 0.088*cos6)*sin5*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 + 0.0825*sin3)*sin1, 
        ((-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*cos2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*sin2)*sin1 + ((0.207*sin6 + 0.088*cos6)*sin5*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 + 0.0825*sin3)*cos1, 
        -(-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*sin2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*cos2 + 0.333], dtype='float')
    
        return A


    def get_angles_from_pose(self, theta_init, A_target, step=0.01, err_tol=1e-3):
        ''' Get joint angles for a target pose. Utilizing pre-computed expressions using DH-parameters.
            
            Parameters:
                theta_init (List): Containing 7 joint angles used to init the iterative search
                A_target (Matrix): Matrix encoding target pose
                step (float): Step width for iteration
                err_tol (float): Error tolerance for returned angles

            Returns:
                A (Matrix): Pose matrix
                    Position is in A[9:12]
                    Rotation is in np.reshape(A[0:9], (3,3))
                pos_error (float): Cartesian position error
        '''
        

        # Settings for divergence detection 
        n_last_entries  = 20                 # n past entries used for detection
        n_allowed_fails = 200                # n consequtive iterations that are allowed before abortion


        delta_list_sqsum    = []            # Sum of squared delta_A entries
        delta_list_max      = []            # Max of delta_A matrix
        n_failed_iterations = 0             # n of failed iterations (non converging)
        

        A_current = self.get_pose_from_angles(theta_init)
        delta_A = (A_target - A_current)
        theta   = theta_init

        while np.linalg.norm(delta_A) > err_tol:
            # TODO: Replace abortion criteria with cartesian equivalent
            J_theta = self.get_J(theta)
            
            # multiplying by step_size to interpolate between current and target pose
            delta_theta = np.matmul(np.linalg.pinv(J_theta), delta_A * step)
            theta       = theta + delta_theta
            
            A_current   = self.get_pose_from_angles(theta)
            delta_A     = (A_target - A_current)

            # Detect divergence
            delta_sqsum = sum(delta_A**2)
            delta_max   = max(abs(delta_A))

            if delta_sqsum >= sum(delta_list_sqsum[-n_last_entries:])/n_last_entries and len(delta_list_sqsum) >= n_last_entries:
                delta_list_sqsum.append(delta_sqsum)
                n_failed_iterations += 1
                print("Warning: IK (square sum) not converging")
                print(delta_list_sqsum[-5:])

            elif delta_max >= sum(delta_list_max[-n_last_entries:])/n_last_entries and len(delta_list_max) >= n_last_entries:
                delta_list_max.append(delta_max)
                n_failed_iterations += 1
                print("Warning: IK (max value) not converging")
                print(delta_list_max[-5:])

            else:
                delta_list_sqsum.append(delta_sqsum)
                delta_list_max.append(delta_max)
                n_failed_iterations = 0

            # Detect too many failed consecutive iterations
            if n_failed_iterations > n_allowed_fails:
                 print("Warning: Too many failed consecutive iterations. Aborting IK")
                 self.debug = True      # Enable to show plot
                 break

            pass
        
        # Show delta stats progress
        if self.debug:
            iteration1   = range(len(delta_list_sqsum))
            iteration2   = range(len(delta_list_max))

            fig, ax1    = plt.subplots()
            ax1.set_xlabel('Iteration')
            ax1.set_ylabel('Squared sum error')
            ax1.plot(iteration1, delta_list_sqsum, color='blue', label='Squared sum')
            l1, label1  = ax1.get_legend_handles_labels()
            ax1.tick_params(axis='y', labelcolor='blue')
            ax1.ticklabel_format(axis='y', style='sci', scilimits=(0,0))

            ax2         = ax1.twinx()
            ax2.set_ylabel('Maximum error')
            ax2.plot(iteration2, delta_list_max, color="red", label='Maximum')
            l2, label2  = ax2.get_legend_handles_labels()
            ax2.tick_params(axis='y', labelcolor='red')
            ax2.ticklabel_format(axis='y', style='sci', scilimits=(0,0))

            ax2.legend(l1 + l2, label1 + label2)
            fig.suptitle('Convergence Behaviour')
            plt.grid()
            plt.show()

        pos_error   = np.linalg.norm(delta_A[9:12])

        return theta, pos_error


    def get_A_2(self, theta_target):
        # obsolete, just here for reference
        
        cos_target  = [np.cos(q) for q in theta_target]
        sin_target  = [np.sin(q) for q in theta_target]
         
        A_lamb_explicit = lambda _0,_1: (lambda c1,c2,c3,c4,c5,c6,c7,s1,s2,s3,s4,s5,s6,s7: (np.array([[c1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7))) - s1*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6))], [c1*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)) + s1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)))], [c2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)) - s2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5))], [c1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5))) - s1*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7))], [c1*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) + s1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)))], [c2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)) - s2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7))], [c1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6)) - s1*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4))], [c1*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4)) + s1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6))], [c2*(-c4*c6 + c5*s4*s6) - s2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6)], [c1*(c2*(c3*(c4*c5*(0.088*c6 + 0.207*s6) - 0.0825*c4 - s4*(-0.207*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.207*s6)) + s2*(c4*(-0.207*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.207*s6) - 0.0825*s4 + 0.316)) - s1*(c3*s5*(0.088*c6 + 0.207*s6) + s3*(c4*c5*(0.088*c6 + 0.207*s6) - 0.0825*c4 - s4*(-0.207*c6 + 0.088*s6 + 0.384)) + 0.0825*s3)], [c1*(c3*s5*(0.088*c6 + 0.207*s6) + s3*(c4*c5*(0.088*c6 + 0.207*s6) - 0.0825*c4 - s4*(-0.207*c6 + 0.088*s6 + 0.384)) + 0.0825*s3) + s1*(c2*(c3*(c4*c5*(0.088*c6 + 0.207*s6) - 0.0825*c4 - s4*(-0.207*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.207*s6)) + s2*(c4*(-0.207*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.207*s6) - 0.0825*s4 + 0.316))], [c2*(c4*(-0.207*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.207*s6) - 0.0825*s4 + 0.316) - s2*(c3*(c4*c5*(0.088*c6 + 0.207*s6) - 0.0825*c4 - s4*(-0.207*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.207*s6)) + 0.333]])))(_0[0],_0[1],_0[2],_0[3],_0[4],_0[5],_0[6],_1[0],_1[1],_1[2],_1[3],_1[4],_1[5],_1[6])
        A = np.array(A_lamb_explicit(cos_target, sin_target))
        A = A[:,0]

        return A


    def get_J(self,theta):
        # current_trigo: sin and cos of actual theta's
        sin1, sin2, sin3, sin4, sin5, sin6, sin7 = sin(theta[0]), sin(theta[1]), sin(theta[2]), sin(theta[3]), sin(theta[4]), sin(theta[5]), sin(theta[6])
        cos1, cos2, cos3, cos4, cos5, cos6, cos7 = cos(theta[0]), cos(theta[1]), cos(theta[2]), cos(theta[3]), cos(theta[4]), cos(theta[5]), cos(theta[6])
        
        J =np.array([[-((((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*cos2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*sin2)*sin1 + (-((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3)*cos1, (-(((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*sin2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*cos2)*cos1, (-((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3)*cos1*cos2 + (-((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (sin5*cos6*cos7 - sin7*cos5)*sin3)*sin1, -(-(sin5*sin7 + cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*sin1*sin3 + ((-(sin5*sin7 + cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*cos2*cos3 + ((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin2)*cos1, (((-sin5*sin7 - cos5*cos6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3*cos4)*cos2 + (-sin5*cos6*cos7 + sin7*cos5)*sin2*sin4)*cos1 + (-(sin5*sin7 + cos5*cos6*cos7)*cos3 - (-sin5*cos6*cos7 + sin7*cos5)*sin3*cos4)*sin1, (((-sin4*cos6*cos7 - sin6*cos4*cos5*cos7)*cos3 + sin3*sin5*sin6*cos7)*cos2 + (-sin4*sin6*cos5*cos7 + cos4*cos6*cos7)*sin2)*cos1 + (-(-sin4*cos6*cos7 - sin6*cos4*cos5*cos7)*sin3 + sin5*sin6*cos3*cos7)*sin1, ((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*cos1 + (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 - (-sin5*sin7*cos6 - cos5*cos7)*cos3)*sin1], 
        [((((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*cos2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*sin2)*cos1 - (((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3)*sin1, (-(((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3)*sin2 + ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*cos2)*sin1, (-((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3)*sin1*cos2 + (((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 - (sin5*cos6*cos7 - sin7*cos5)*sin3)*cos1, (-(sin5*sin7 + cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*sin3*cos1 + ((-(sin5*sin7 + cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*cos2*cos3 + ((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin2)*sin1, (((-sin5*sin7 - cos5*cos6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3*cos4)*cos2 + (-sin5*cos6*cos7 + sin7*cos5)*sin2*sin4)*sin1 + ((sin5*sin7 + cos5*cos6*cos7)*cos3 + (-sin5*cos6*cos7 + sin7*cos5)*sin3*cos4)*cos1, (((-sin4*cos6*cos7 - sin6*cos4*cos5*cos7)*cos3 + sin3*sin5*sin6*cos7)*cos2 + (-sin4*sin6*cos5*cos7 + cos4*cos6*cos7)*sin2)*sin1 + ((-sin4*cos6*cos7 - sin6*cos4*cos5*cos7)*sin3 - sin5*sin6*cos3*cos7)*cos1, ((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*sin1 + (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 + (-sin5*sin7*cos6 - cos5*cos7)*cos3)*cos1], 
        [0, (-((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos3 - (-sin5*cos6*cos7 + sin7*cos5)*sin3)*cos2 - ((sin5*sin7 + cos5*cos6*cos7)*sin4 + sin6*cos4*cos7)*sin2, (((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3)*sin2, -(-(sin5*sin7 + cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*sin2*cos3 + ((sin5*sin7 + cos5*cos6*cos7)*cos4 - sin4*sin6*cos7)*cos2, (-(-sin5*sin7 - cos5*cos6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3*cos4)*sin2 + (-sin5*cos6*cos7 + sin7*cos5)*sin4*cos2, (-(-sin4*cos6*cos7 - sin6*cos4*cos5*cos7)*cos3 - sin3*sin5*sin6*cos7)*sin2 + (-sin4*sin6*cos5*cos7 + cos4*cos6*cos7)*cos2, (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 - (sin5*sin7*cos6 + cos5*cos7)*sin3)*sin2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*cos2], 
        [-((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*sin1 + (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 + (sin5*sin7*cos6 + cos5*cos7)*cos3)*cos1, (-(((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*sin2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*cos2)*cos1, (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 + (sin5*sin7*cos6 + cos5*cos7)*cos3)*cos1*cos2 + (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (-sin5*sin7*cos6 - cos5*cos7)*sin3)*sin1, -(-(sin5*cos7 - sin7*cos5*cos6)*sin4 + sin6*sin7*cos4)*sin1*sin3 + ((-(sin5*cos7 - sin7*cos5*cos6)*sin4 + sin6*sin7*cos4)*cos2*cos3 + ((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin2)*cos1, (((-sin5*cos7 + sin7*cos5*cos6)*sin3 + (sin5*sin7*cos6 + cos5*cos7)*cos3*cos4)*cos2 + (sin5*sin7*cos6 + cos5*cos7)*sin2*sin4)*cos1 + (-(sin5*cos7 - sin7*cos5*cos6)*cos3 - (sin5*sin7*cos6 + cos5*cos7)*sin3*cos4)*sin1, (((sin4*sin7*cos6 + sin6*sin7*cos4*cos5)*cos3 - sin3*sin5*sin6*sin7)*cos2 + (sin4*sin6*sin7*cos5 - sin7*cos4*cos6)*sin2)*cos1 + (-(sin4*sin7*cos6 + sin6*sin7*cos4*cos5)*sin3 - sin5*sin6*sin7*cos3)*sin1, ((((-sin5*sin7 - cos5*cos6*cos7)*cos4 + sin4*sin6*cos7)*cos3 + (sin5*cos6*cos7 - sin7*cos5)*sin3)*cos2 + ((-sin5*sin7 - cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*sin2)*cos1 + (-((-sin5*sin7 - cos5*cos6*cos7)*cos4 + sin4*sin6*cos7)*sin3 - (-sin5*cos6*cos7 + sin7*cos5)*cos3)*sin1], 
        [((((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2)*cos1 - (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 - (sin5*sin7*cos6 + cos5*cos7)*cos3)*sin1, (-(((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3)*sin2 + ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*cos2)*sin1, (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 + (sin5*sin7*cos6 + cos5*cos7)*cos3)*sin1*cos2 + (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 - (-sin5*sin7*cos6 - cos5*cos7)*sin3)*cos1, (-(sin5*cos7 - sin7*cos5*cos6)*sin4 + sin6*sin7*cos4)*sin3*cos1 + ((-(sin5*cos7 - sin7*cos5*cos6)*sin4 + sin6*sin7*cos4)*cos2*cos3 + ((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin2)*sin1, (((-sin5*cos7 + sin7*cos5*cos6)*sin3 + (sin5*sin7*cos6 + cos5*cos7)*cos3*cos4)*cos2 + (sin5*sin7*cos6 + cos5*cos7)*sin2*sin4)*sin1 + ((sin5*cos7 - sin7*cos5*cos6)*cos3 + (sin5*sin7*cos6 + cos5*cos7)*sin3*cos4)*cos1, (((sin4*sin7*cos6 + sin6*sin7*cos4*cos5)*cos3 - sin3*sin5*sin6*sin7)*cos2 + (sin4*sin6*sin7*cos5 - sin7*cos4*cos6)*sin2)*sin1 + ((sin4*sin7*cos6 + sin6*sin7*cos4*cos5)*sin3 + sin5*sin6*sin7*cos3)*cos1, ((((-sin5*sin7 - cos5*cos6*cos7)*cos4 + sin4*sin6*cos7)*cos3 + (sin5*cos6*cos7 - sin7*cos5)*sin3)*cos2 + ((-sin5*sin7 - cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*sin2)*sin1 + (((-sin5*sin7 - cos5*cos6*cos7)*cos4 + sin4*sin6*cos7)*sin3 + (-sin5*cos6*cos7 + sin7*cos5)*cos3)*cos1], 
        [0, (-((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos3 - (sin5*sin7*cos6 + cos5*cos7)*sin3)*cos2 - ((sin5*cos7 - sin7*cos5*cos6)*sin4 - sin6*sin7*cos4)*sin2, (((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*sin3 - (sin5*sin7*cos6 + cos5*cos7)*cos3)*sin2, -(-(sin5*cos7 - sin7*cos5*cos6)*sin4 + sin6*sin7*cos4)*sin2*cos3 + ((sin5*cos7 - sin7*cos5*cos6)*cos4 + sin4*sin6*sin7)*cos2, (-(-sin5*cos7 + sin7*cos5*cos6)*sin3 - (sin5*sin7*cos6 + cos5*cos7)*cos3*cos4)*sin2 + (sin5*sin7*cos6 + cos5*cos7)*sin4*cos2, (-(sin4*sin7*cos6 + sin6*sin7*cos4*cos5)*cos3 + sin3*sin5*sin6*sin7)*sin2 + (sin4*sin6*sin7*cos5 - sin7*cos4*cos6)*cos2, (-((-sin5*sin7 - cos5*cos6*cos7)*cos4 + sin4*sin6*cos7)*cos3 - (sin5*cos6*cos7 - sin7*cos5)*sin3)*sin2 + ((-sin5*sin7 - cos5*cos6*cos7)*sin4 - sin6*cos4*cos7)*cos2], 
        [-(((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*cos2 + (sin4*sin6*cos5 - cos4*cos6)*sin2)*sin1 + (-(sin4*cos6 + sin6*cos4*cos5)*sin3 - sin5*sin6*cos3)*cos1, (-((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*sin2 + (sin4*sin6*cos5 - cos4*cos6)*cos2)*cos1, (-(sin4*cos6 + sin6*cos4*cos5)*sin3 - sin5*sin6*cos3)*cos1*cos2 + (-(sin4*cos6 + sin6*cos4*cos5)*cos3 + sin3*sin5*sin6)*sin1, ((sin4*cos6 + sin6*cos4*cos5)*sin2 + (-sin4*sin6*cos5 + cos4*cos6)*cos2*cos3)*cos1 - (-sin4*sin6*cos5 + cos4*cos6)*sin1*sin3, ((-sin3*sin6*cos5 - sin5*sin6*cos3*cos4)*cos2 - sin2*sin4*sin5*sin6)*cos1 + (sin3*sin5*sin6*cos4 - sin6*cos3*cos5)*sin1, (((-sin4*sin6 + cos4*cos5*cos6)*cos3 - sin3*sin5*cos6)*cos2 + (sin4*cos5*cos6 + sin6*cos4)*sin2)*cos1 + (-(-sin4*sin6 + cos4*cos5*cos6)*sin3 - sin5*cos3*cos6)*sin1, 0], 
        [(((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*cos2 + (sin4*sin6*cos5 - cos4*cos6)*sin2)*cos1 - ((sin4*cos6 + sin6*cos4*cos5)*sin3 + sin5*sin6*cos3)*sin1, (-((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*sin2 + (sin4*sin6*cos5 - cos4*cos6)*cos2)*sin1, (-(sin4*cos6 + sin6*cos4*cos5)*sin3 - sin5*sin6*cos3)*sin1*cos2 + ((sin4*cos6 + sin6*cos4*cos5)*cos3 - sin3*sin5*sin6)*cos1, ((sin4*cos6 + sin6*cos4*cos5)*sin2 + (-sin4*sin6*cos5 + cos4*cos6)*cos2*cos3)*sin1 + (-sin4*sin6*cos5 + cos4*cos6)*sin3*cos1, ((-sin3*sin6*cos5 - sin5*sin6*cos3*cos4)*cos2 - sin2*sin4*sin5*sin6)*sin1 + (-sin3*sin5*sin6*cos4 + sin6*cos3*cos5)*cos1, (((-sin4*sin6 + cos4*cos5*cos6)*cos3 - sin3*sin5*cos6)*cos2 + (sin4*cos5*cos6 + sin6*cos4)*sin2)*sin1 + ((-sin4*sin6 + cos4*cos5*cos6)*sin3 + sin5*cos3*cos6)*cos1, 0], 
        [0, (-(sin4*cos6 + sin6*cos4*cos5)*cos3 + sin3*sin5*sin6)*cos2 - (sin4*sin6*cos5 - cos4*cos6)*sin2, ((sin4*cos6 + sin6*cos4*cos5)*sin3 + sin5*sin6*cos3)*sin2, (sin4*cos6 + sin6*cos4*cos5)*cos2 - (-sin4*sin6*cos5 + cos4*cos6)*sin2*cos3, (sin3*sin6*cos5 + sin5*sin6*cos3*cos4)*sin2 - sin4*sin5*sin6*cos2, (-(-sin4*sin6 + cos4*cos5*cos6)*cos3 + sin3*sin5*cos6)*sin2 + (sin4*cos5*cos6 + sin6*cos4)*cos2, 0], 
        [-((-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*cos2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*sin2)*sin1 + (-(0.207*sin6 + 0.088*cos6)*sin5*cos3 - ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 - 0.0825*sin3)*cos1, (-(-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*sin2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*cos2)*cos1, ((-0.207*sin6 - 0.088*cos6)*sin5*cos3 - ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 - 0.0825*sin3)*cos1*cos2 + ((0.207*sin6 + 0.088*cos6)*sin3*sin5 - ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 - 0.0825*cos3)*sin1, ((-(0.207*sin6 + 0.088*cos6)*sin4*cos5 + (-0.088*sin6 + 0.207*cos6 - 0.384)*cos4 + 0.0825*sin4)*cos2*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin2)*cos1 - (-(0.207*sin6 + 0.088*cos6)*sin4*cos5 + (-0.088*sin6 + 0.207*cos6 - 0.384)*cos4 + 0.0825*sin4)*sin1*sin3, (((-0.207*sin6 - 0.088*cos6)*sin3*cos5 - (0.207*sin6 + 0.088*cos6)*sin5*cos3*cos4)*cos2 - (0.207*sin6 + 0.088*cos6)*sin2*sin4*sin5)*cos1 + ((0.207*sin6 + 0.088*cos6)*sin3*sin5*cos4 - (0.207*sin6 + 0.088*cos6)*cos3*cos5)*sin1, ((((-0.207*sin6 - 0.088*cos6)*sin4 + (-0.088*sin6 + 0.207*cos6)*cos4*cos5)*cos3 + (0.088*sin6 - 0.207*cos6)*sin3*sin5)*cos2 + ((-0.088*sin6 + 0.207*cos6)*sin4*cos5 + (0.207*sin6 + 0.088*cos6)*cos4)*sin2)*cos1 + (-((-0.207*sin6 - 0.088*cos6)*sin4 + (-0.088*sin6 + 0.207*cos6)*cos4*cos5)*sin3 - (-0.088*sin6 + 0.207*cos6)*sin5*cos3)*sin1, 0], 
        [((-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*cos2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*sin2)*cos1 - ((0.207*sin6 + 0.088*cos6)*sin5*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 + 0.0825*sin3)*sin1, (-(-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*sin2 + ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*cos2)*sin1, ((-0.207*sin6 - 0.088*cos6)*sin5*cos3 - ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 - 0.0825*sin3)*sin1*cos2 + (-(0.207*sin6 + 0.088*cos6)*sin3*sin5 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 + 0.0825*cos3)*cos1, ((-(0.207*sin6 + 0.088*cos6)*sin4*cos5 + (-0.088*sin6 + 0.207*cos6 - 0.384)*cos4 + 0.0825*sin4)*cos2*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin2)*sin1 + (-(0.207*sin6 + 0.088*cos6)*sin4*cos5 + (-0.088*sin6 + 0.207*cos6 - 0.384)*cos4 + 0.0825*sin4)*sin3*cos1, (((-0.207*sin6 - 0.088*cos6)*sin3*cos5 - (0.207*sin6 + 0.088*cos6)*sin5*cos3*cos4)*cos2 - (0.207*sin6 + 0.088*cos6)*sin2*sin4*sin5)*sin1 + (-(0.207*sin6 + 0.088*cos6)*sin3*sin5*cos4 + (0.207*sin6 + 0.088*cos6)*cos3*cos5)*cos1, ((((-0.207*sin6 - 0.088*cos6)*sin4 + (-0.088*sin6 + 0.207*cos6)*cos4*cos5)*cos3 + (0.088*sin6 - 0.207*cos6)*sin3*sin5)*cos2 + ((-0.088*sin6 + 0.207*cos6)*sin4*cos5 + (0.207*sin6 + 0.088*cos6)*cos4)*sin2)*sin1 + (((-0.207*sin6 - 0.088*cos6)*sin4 + (-0.088*sin6 + 0.207*cos6)*cos4*cos5)*sin3 + (-0.088*sin6 + 0.207*cos6)*sin5*cos3)*cos1, 0], 
        [0, ((0.207*sin6 + 0.088*cos6)*sin3*sin5 - ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos3 - 0.0825*cos3)*cos2 - ((0.207*sin6 + 0.088*cos6)*sin4*cos5 + (0.088*sin6 - 0.207*cos6 + 0.384)*cos4 - 0.0825*sin4 + 0.316)*sin2, (-(-0.207*sin6 - 0.088*cos6)*sin5*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*sin3 + 0.0825*sin3)*sin2, -(-(0.207*sin6 + 0.088*cos6)*sin4*cos5 + (-0.088*sin6 + 0.207*cos6 - 0.384)*cos4 + 0.0825*sin4)*sin2*cos3 + ((0.207*sin6 + 0.088*cos6)*cos4*cos5 - (0.088*sin6 - 0.207*cos6 + 0.384)*sin4 - 0.0825*cos4)*cos2, (-(-0.207*sin6 - 0.088*cos6)*sin3*cos5 + (0.207*sin6 + 0.088*cos6)*sin5*cos3*cos4)*sin2 - (0.207*sin6 + 0.088*cos6)*sin4*sin5*cos2, (-((-0.207*sin6 - 0.088*cos6)*sin4 + (-0.088*sin6 + 0.207*cos6)*cos4*cos5)*cos3 - (0.088*sin6 - 0.207*cos6)*sin3*sin5)*sin2 + ((-0.088*sin6 + 0.207*cos6)*sin4*cos5 + (0.207*sin6 + 0.088*cos6)*cos4)*cos2, 0]], dtype='float')
        
        return J


# for testing an debugging

if __name__ == '__main__':

    profiler_on = False

    if profiler_on:
        import cProfile, pstats
        profiler = cProfile.Profile()
        profiler.enable()

    
    kinematics  = robot_kinematics(debug=True)

    theta_init  = np.array([ 0,  0,  0,  0,  0,  0,  0])    # Theta used to init iterative search (in degree)
    theta_delta = np.array([20, 20, 20, 20, 20, 20, 20])    # Delta from init theta to target theta (in degree)
    theta_target = [np.deg2rad(theta_init[i] + theta_delta[i]) for i in range(7)]

    A_current   = kinematics.get_pose_from_angles(theta_init)
    A_target    = kinematics.get_pose_from_angles(theta_target)
    print(A_target)

    q_ik, error = kinematics.get_angles_from_pose(theta_init, A_target)
    print(np.rad2deg(q_ik))
    print(f"Position error: {error}")


    if profiler_on:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.print_stats(20)


    # Result of code profiling
    # Unsurprisingly get_J and get_A consume most of the time
    #
    #   ncalls    tottime  percall cumtime  percall filename:lineno(function)
    #         1     0.004    0.004   40.203   40.203 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:22(inverse_kinematic)
    #        55     0.984    0.018   23.418    0.426 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:58(get_J)
    # 140479/137920 1.821    0.000   19.936    0.000 /home/rnm/.local/lib/python3.8/site-packages/sympy/core/decorators.py:58(__sympifyit_wrapper)
    #        57     0.206    0.004   16.862    0.296 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:37(get_A)
    # 26056/17553   0.486    0.000   14.918    0.001 /home/rnm/.local/lib/python3.8/site-packages/sympy/core/cache.py:67(wrapper)
    #     16447     0.424    0.000   14.187    0.001 /home/rnm/.local/lib/python3.8/site-packages/sympy/core/decorators.py:224(_func)
    #     94532     2.559    0.000    9.720    0.000 /home/rnm/.local/lib/python3.8/site-packages/sympy/core/numbers.py:1293(__mul__)
    #
    # After mod
    #     ncalls  tottime  percall  cumtime  percall filename:lineno(function)
    #         1    0.007    0.007    0.200    0.200 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:23(inverse_kinematic)
    #        55    0.002    0.000    0.085    0.002 <__array_function__ internals>:2(pinv)
    #    279/59    0.007    0.000    0.084    0.001 {built-in method numpy.core._multiarray_umath.implement_array_function}
    #        55    0.077    0.001    0.082    0.001 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:71(get_J)
    #        55    0.012    0.000    0.081    0.001 /usr/lib/python3/dist-packages/numpy/linalg/linalg.py:1890(pinv)
    #        55    0.002    0.000    0.034    0.001 <__array_function__ internals>:2(svd)
    #        55    0.016    0.000    0.030    0.001 /usr/lib/python3/dist-packages/numpy/linalg/linalg.py:1468(svd)
    #        57    0.022    0.000    0.024    0.000 /home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/kinematic.py:50(get_A)
