#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt
from DH_creator import DhCreator

class robot_kinematics:
    def __init__(self, debug=False):
        ''' Parameters:
                debug (bool): Enables debug behavior
        '''
        # flags
        self.debug = debug                      # flag to (de)activate plots to controll divergence/ konvergence
        
        # instances
        dh_creator = DhCreator()                
        self.A_lamb = dh_creator.get_A_lamb()   # load lambdafunction of A which is calculated with the help of a DH-Matrix
        self.J_lamb = dh_creator.get_J_lamb()   # load lambdafunction of J which is calculated with the help of the lambdafunction of A

    def get_pose_from_angles(self, theta_target):
        ''' Get pose matrix A from joint angles. Utilizing pre-computed expressions using DH-parameters.
            
            Parameters:
                theta (List): Containing 7 joint angles

            Returns:
                A (Matrix): Pose matrix
                    Position is in A[9:12]
                    Rotation is in np.reshape(A[0:9], (3,3))
        '''
        cos_target  = [np.cos(q) for q in theta_target] # calculate cos(q_i) for each q1 - q7 of theta
        sin_target  = [np.sin(q) for q in theta_target] # calculate sin(q_i) for each q1 - q7 of theta
         
        A = np.array(self.A_lamb(cos_target, sin_target))  # calculate A
        A = A[:,0]                                         # reshape A 

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
        n_last_entries  = 500                 # n past entries used for detection
        n_allowed_fails = 10000               # n consequtive iterations that are allowed before abortion


        delta_list_sqsum    = []            # Sum of squared delta_A entries
        delta_list_max      = []            # Max of delta_A matrix
        n_failed_iterations = 0             # n of failed iterations (non converging)
        
        counter = 0                         # counter for debugging purpose (iterations per completet calculation)

        A_current = self.get_pose_from_angles(theta_init)   # calculate intial A
        delta_A = (A_target - A_current)                    # calculate intial delta A
        theta   = theta_init                                # set inital theta

        while np.abs(delta_A).max() > err_tol:

            J_theta = self.get_J(theta)
            
            # multiplying by step_size to interpolate between current and target pose
            delta_theta = np.matmul(np.linalg.pinv(J_theta), delta_A * step)
            theta       = theta + delta_theta
            
            A_current   = self.get_pose_from_angles(theta)
            delta_A     = (A_target - A_current)

            # Detect divergence
            delta_sqsum = sum(delta_A**2)
            delta_max   = max(abs(delta_A))
            counter += 1
            if delta_sqsum >= sum(delta_list_sqsum[-n_last_entries:])/n_last_entries and len(delta_list_sqsum) >= n_last_entries:
                delta_list_sqsum.append(delta_sqsum)
                n_failed_iterations += 1
                #print("Warning: IK (square sum) not converging")
                #print(delta_list_sqsum[-5:])

            elif delta_max >= sum(delta_list_max[-n_last_entries:])/n_last_entries and len(delta_list_max) >= n_last_entries:
                delta_list_max.append(delta_max)
                n_failed_iterations += 1
                #print("Warning: IK (max value) not converging")
                #print(delta_list_max[-5:])

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
        print(counter)
        return theta, pos_error




    def get_J(self,theta):
        # current_trigo: sin and cos of actual theta's
        cos_target  = [np.cos(q) for q in theta]
        sin_target  = [np.sin(q) for q in theta]
        J = np.array(self.J_lamb(cos_target, sin_target))
        return J


# for testing an debugging

if __name__ == '__main__':

    profiler_on = False

    if profiler_on:
        import cProfile, pstats
        profiler = cProfile.Profile()
        profiler.enable()

    
    kinematics  = robot_kinematics(debug=True)

    theta_init  = np.array([ 0.0,0.0,0.0,-1.0,0.0,1.0,0.0])    # Theta used to init iterative search (in degree)
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
