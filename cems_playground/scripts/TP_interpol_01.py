import csv
import os
from typing import List
import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial

class TrajectorySegment:

    def __init__(self,  seg_id : int, poly_coef : List, start_t : float, end_t : float, 
                        start_pos : float, end_pos : float, is_waypoint : bool):
        ''' TODO Docstring 
            is_waypoint (bool): start_pos is waypoint
        '''
        self.id_num     = seg_id
        self.poly_coef  = poly_coef
        self.start_t    = start_t
        self.end_t      = end_t
        self.start_pos  = start_pos
        self.end_pos    = end_pos
        self.is_waypoint= is_waypoint

    def get_values_for_window(self, derivative=0, step=0.001, sig_dec=3):
        ''' Returns x and y values for the n-th derivative of the polynomial in the window
            between start_t and end_t
            Parameters:
                derivative (int): Indicates the desired derivative of the polynomial
                step (float): x step width in seconds, must be <= 1
                sig_dec (int): Round to significant decimal place
            Returns:
                x (List): x values in seconds
                y (List): y values
        '''
        start   = np.round(self.start_t, sig_dec)
        end     = np.round(self.end_t, sig_dec)
        x       = np.arange(start, end, step)
        poly    = Polynomial(self.poly_coef)
        y       = poly.deriv(derivative)(x - start)

        return x, y


class Trajectory:

    trajectories    = []    # list of all trajectories

    def __init__(self, joint_id : int, initial_time : float):
        self.joint_id       = joint_id
        self.segment_list   = []                #type: List[TrajectorySegment]
        self.time_list      = [initial_time]

        self.trajectories.append(self)
        
    def register_segment(self, poly_coef, duration, start_pos, end_pos, is_waypoint):
        ''' TODO Docstring '''
        if duration == 0 or duration == np.inf or duration == np.nan:
            assert 0, "Duration seems wrong"

        segment = TrajectorySegment(seg_id      = len(self.segment_list),
                                    poly_coef   = poly_coef,
                                    start_t     = self.time_list[-1],
                                    end_t       = self.time_list[-1] + duration,
                                    start_pos   = start_pos,
                                    end_pos     = end_pos,
                                    is_waypoint = is_waypoint)

        self.segment_list.append(segment)
        self.time_list.append(segment.end_t)

    def get_all_segments(self, derivative=0, step=0.001, sig_dec=3):
        ''' Stitches all available segments together and returns 
            an x and y list for all segemnts and an array with all
            time_offsets
        '''
        x       = []
        y       = []
        pos     = []
        is_wp   = []
        
        for segment in self.segment_list:   
            x_new, y_new = segment.get_values_for_window(derivative, step, sig_dec)

            [x.append(x_n) for x_n in x_new]
            [y.append(y_n) for y_n in y_new]
            pos.append(segment.start_pos)
            is_wp.append(segment.is_waypoint)
        
        pos.append(self.segment_list[-1].end_pos)
        is_wp.append(True)

        return x, y, (self.time_list, pos, is_wp)


def get_list_from_csv(dir_name, file_name):
    ''' Get a list of lists from a csv file
        dir_name should be relative to the location of util.py
    '''

    output  = []
    with open((os.path.join(os.path.dirname(__file__), dir_name, file_name)), newline="") as csv_file:
        reader  = csv.reader(csv_file)
        for row in reader:
            row_float = [float(entry) for entry in row]
            output.append(row_float)

    return output

def printm(message):
    print(message)

limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
            "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
            "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
            "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
            "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
            "p_vel_max" :   1.7000,
            "p_acc_max" :  13.0000,
            "p_jrk_max" : 6500.000 }

joint_num   = 3

# Waypoints here in degree (actually in radians)
waypoints   = get_list_from_csv("", "waypoints_02.csv")
waypoints   = np.array(waypoints)
waypoints_t = waypoints.transpose()

# Convert limits to degree, for easier reading
keys    = ["q_pos_max", "q_pos_min", "q_vel_max", "q_acc_max" ,"q_jrk_max"]
for key in keys:
    limits[key] = np.rad2deg(limits[key])


# START-------------------------------------------------------
def get_quintic_params(t, pos, vel, acc):
    ''' Get parameters for quintic polynomial from time, position, velocity 
        and acceleration values.
        For each individual calculation, the start time is set to 0.
    '''
    qpoly_params    = []
    for i in range(1, len(t)):
        ip  = i - 1
        t2  = t[i] - t[ip]                     

        a   = np.array([[1,  0,       0,       0,        0,        0],
                        [0,  1,       0,       0,        0,        0],
                        [0,  0,       2,       0,        0,        0],
                        [1, t2,   t2**2,   t2**3,    t2**4,    t2**5],
                        [0,  1, 2*t2   , 3*t2**2,  4*t2**3,  5*t2**4],
                        [0,  0,       2, 6*t2   , 12*t2**2, 20*t2**3]])
        b   = np.array([pos[ip], vel[ip], acc[ip], pos[i], vel[i], acc[i]])
        p   = np.linalg.solve(a, b)
        qpoly_params.append(p)
    
    return qpoly_params

def get_initial_velocities(waypoints_t, limits):
    ''' Given a waypoint matrix [joint][waypoint] a corresponding velocity matrix is returned.
        Velocities are given in the following way:
            - First and last velocities are always set to 0
            - if prev_waypoint < curr_waypoint < next_waypoint: curr_vel = + max_vel
            - if prev_waypoint > curr_waypoint > next_waypoint: curr_vel = - max_vel
            - else: curr_vel = 0
    '''
    
    joints_vel  = np.zeros(waypoints_t.shape)
    wp          = waypoints_t

    # Iterate over joints
    for j in range(wp.shape[0]):
        max_vel = limits['q_vel_max'][j]
        for k in range(1, wp.shape[1] - 1):
            if   wp[j, k - 1] < wp[j, k] < wp[j, k + 1]: joints_vel[j, k] = max_vel
            elif wp[j, k - 1] > wp[j, k] > wp[j, k + 1]: joints_vel[j, k] = - max_vel
            else: joints_vel[j, k] = 0

            # TODO override here for debug
            joints_vel[j, k] = max_vel
            pass

    joints_vel[:, 0] = 0
    joints_vel[:,-1] = 0

    return joints_vel

def plot_trajectory_waypoint(poly_params_pos, time_offset, pos):

    fig, ax = plt.subplots()

    x       = np.linspace(0, time_offset[-1], 100)  # in seconds
    labels  = ['ramp up', 'cruise', 'ramp down', 'hold']

    # Plot poly lines
    for qpoly_param, t, label in zip(poly_params_pos, time_offset, labels) :
        poly    = Polynomial(qpoly_param)
        y       = [poly(i - t) for i in x]
        ax.plot(x, y, label=label)

    # Plot vertical lines for quintic segments
    for t in time_offset:
        ax.axvline(t, color='black', lw=1, linestyle=':')
    
    # Plot dots to show waypoints
    for i, (t, p) in enumerate(zip(time_offset, pos)):
        if i == 0 or i == len(time_offset) - 1:
            ax.plot(t, p, 'o', color='black', label='Waypoints' if i == 0 else "")
        else:
            ax.plot(t, p, 'x', color='black', label='Control Points' if i == 1 else "")


    ax.set_xlim([-0.05, time_offset[-1] + 0.05])
    ax.set_ylim([min(pos) - 5, max(pos) + 5])
    ax.set_title('Quintic poly interpolation between two waypoints')
    ax.legend()
    plt.show()

def plot_trajectory_waypoint_limits(params_pos, params_vel, params_acc, params_jrk, time_offset, pos, limits, joint_id=0):

    fig, axs= plt.subplots(4, 1, sharex='col')

    x               = np.ogrid[0: np.round(time_offset[-1], 4): 0.001]      # Actual sample rate
    x               = np.linspace(0, time_offset[-1], 5000)                 # in seconds
    labels_segments = ['ramp up', 'cruise', 'ramp down', 'hold']
    labels_axis     = [r'Pos $[deg]$', r'Vel $[deg/s]$', r'Acc $[deg/s^2]$', r'Jerk $[deg/s^3]$']     # /TODO: Change to radians for implementation
    poly_params     = [params_pos, params_vel, params_acc, params_jrk]
    max_value       = [limits['q_pos_max'][joint_id],  limits['q_vel_max'][joint_id],  limits['q_acc_max'][joint_id],  limits['q_jrk_max'][joint_id]]
    min_value       = [limits['q_pos_min'][joint_id], -limits['q_vel_max'][joint_id], -limits['q_acc_max'][joint_id], -limits['q_jrk_max'][joint_id]]

    for n in range(len(poly_params)):
        params  = poly_params[n]
        label_a = labels_axis[n]

        # Plot poly lines
        for p, (param, t) in enumerate(zip(params, time_offset)) :
            poly    = Polynomial(param)
            xx      = [i for i in x if i >= t and i <= time_offset[p + 1]]
            y       = [poly(i - t) for i in xx]
            axs[n].plot(xx, y, label=labels_segments[p])
        
        # Plot vertical lines for quintic segments
        for t in time_offset:
            axs[n].axvline(t, color='black', lw=1, linestyle=':')
        
        # Plot horizontal lines for limits
        axs[n].axhline(max_value[n], color='black', lw=2, linestyle='--')
        axs[n].axhline(min_value[n], color='black', lw=2, linestyle='--')

        axs[n].set_xlim([-0.05, time_offset[-1] + 0.05])
        axs[n].set_ylabel(label_a)

    # Plot dots to show waypoints and limit yaxis
    for i, (t, p) in enumerate(zip(time_offset, pos)):
        if i == 0 or i == len(time_offset) - 1:
            axs[0].plot(t, p, 'o', color='black', label='Waypoints' if i == 0 else "")
        else:
            axs[0].plot(t, p, 'x', color='black', label='Control Point' if i == 1 else "")

    axs[0].set_ylim([min(pos) - 5, max(pos) + 5])
    axs[0].set_title('4 segment quintic polynomial interpolation')
    axs[3].legend(bbox_to_anchor =(0.5,-0.8), loc='lower center', ncol=6)

    plt.tight_layout()
    plt.show()

    pass

def plot_trajectory_stitched(traj : Trajectory):
    
    joint_id    = traj.joint_id
    max_value   = [limits['q_pos_max'][joint_id],  limits['q_vel_max'][joint_id],  limits['q_acc_max'][joint_id],  limits['q_jrk_max'][joint_id]]
    min_value   = [limits['q_pos_min'][joint_id], -limits['q_vel_max'][joint_id], -limits['q_acc_max'][joint_id], -limits['q_jrk_max'][joint_id]]

    fig, axs    = plt.subplots(4, 1, sharex='col')
    labels_axis = [r'Pos $[deg]$', r'Vel $[deg/s]$', r'Acc $[deg/s^2]$', r'Jerk $[deg/s^3]$'] 

    for axs_n in range(4):

        x, y, (t, pos, is_wp) = traj.get_all_segments(derivative=axs_n, step=0.00001, sig_dec=5)
        axs[axs_n].plot(x, y)

        wp_count    = 0
        # Draw additional lines and points
        for i, (t_, pos_, is_wp_) in enumerate(zip(t, pos, is_wp)):
            if(is_wp_) :
                axs[axs_n].axvline(t_, color='black', lw=1, linestyle='-')
                if axs_n == 0: 
                    axs[0].plot(t_, pos_, 'o', color='black', label='Waypoint', ms=5)
                    axs[0].annotate(str(wp_count), (t_ + 0.01, pos_ + 1))
                    wp_count = wp_count + 1
            else :
                axs[axs_n].axvline(t_, color='gray', lw=0.5, linestyle='-')
                if axs_n == 0: 
                    axs[0].plot(t_, pos_, 'x', color='black', label='Control Point', ms=5)

        axs[axs_n].axhline(max_value[axs_n], color='black', lw=2, linestyle='--')
        axs[axs_n].axhline(min_value[axs_n], color='black', lw=2, linestyle='--')

        # Ax settings
        axs[axs_n].set_ylabel(labels_axis[axs_n])
        axs[0].set_ylim(min(pos) - 5, max(pos) + 5)

    plt.tight_layout()
    plt.show()

    a = 1
    
def get_test_case(pos_direction, pos_sign, vel_direction, vel_sign, max_vel):
    pos = []
    vel = []

    if pos_direction == "constant":
        pos = [30, 30, 30, 30, 30]
    if pos_direction == "rising":
        pos = [0, 20, 100, 110, 120]
    if pos_direction == "falling":
        pos = [120, 110, 100, 20, 0]
    if pos_direction == "swoop":
        pos = [0, 80, 100, 40, 0]
    if pos_sign == "-":
        pos.reverse()
        pos = - np.array(pos)

    if vel_direction == "zero":
        vel = [0, 0, 0, 0, 0]
    if vel_direction == "constant":
        vel = [0, max_vel, max_vel, max_vel, 0]    
    if vel_direction == "rising":
        vel = [0, max_vel*0.1, max_vel*0.5, max_vel, 0]
    if vel_direction == "falling":
        vel = [0, max_vel, max_vel*0.5, max_vel*0.1, 0]
    if vel_direction == "swoop":
        vel = [0, max_vel*0.5, max_vel, max_vel*0.1, 0]
    if vel_sign == "-":
        vel.reverse()
        vel = - np.array(vel)
    
    printm(f"Running test case: pos_{pos_sign}_{pos_direction}, vel_{vel_sign}_{vel_direction}")

    pos     = np.array([pos])
    vel     = np.array([vel])
    return pos, vel


""" TODO Tests Cases:
    Successfull:
    pos_+_rising, vel_+_constant
    pos_-_rising, vel_+_constant
    pos_+_rising, vel_+_rising
    pos_-_rising, vel_+_rising
    pos_+_rising, vel_+_falling
    pos_-_rising, vel_+_falling
    pos_+_falling, vel_-_constant
    pos_-_falling, vel_-_constant
    pos_+_falling, vel_-_rising
    pos_-_falling, vel_-_rising
    To be done:
    Pos constant, vel 0    
    Pos falling  
    Pos falling
    Pos falling
"""

waypoints_t, joints_vel = get_test_case('falling','+','constant','+', limits['q_vel_max'][0])

# Check if path has invalid values (out of min max bounds)
for j, joint_positions in enumerate(waypoints_t):
    for k, wp in enumerate(joint_positions):
        assert wp >= limits['q_pos_min'][j] and wp <= limits['q_pos_max'][j], f"Waypoint {k} of joint {j} out of q_min_pos, q_max_pos bounds: {wp}"


# Get trajectory parameters for joints--------------------------
joints_time = np.zeros(waypoints_t.shape)
#joints_vel  = np.zeros(waypoints_t.shape)

# Iterate over joints
for j, joint_positions in enumerate(waypoints_t):   

    # Joint parameters
    max_vel     = limits['q_vel_max'][j]
    max_acc     = limits['q_acc_max'][j]
    max_jrk     = limits['q_jrk_max'][j]
    ramp_time   = (np.pi * max_acc) / (2 * max_jrk)    # Time for a full ramp with sinosodial acc profile

    # Initial time
    joints_time[j][0]  = 0
    #joints_vel[j] = [0, max_acc, max_acc, max_acc, 0]

    trajectory_joint0  = Trajectory(0, 0)
    
    # Iterate over positions
    for p in range(1, len(joint_positions)):        
        wp1     = joint_positions[p - 1]    # Previous waypoint
        wp2     = joint_positions[p]        # Current waypoint
        dist    = abs(wp1 - wp2)
        full_ramp_dist  = max_acc * (ramp_time ** 2) + 2 * joints_vel[j][p - 1] * ramp_time

        # Swap velocities in case v2 is smaller than v1 (so we always have a rising velocity for calculations)
        if joints_vel[j, p - 1] < joints_vel[j, p]:
            v1  = joints_vel[j, p - 1]      # Previous velocity     for v1 < v2
            v2  = joints_vel[j, p]          # Current velocity
        else:
            v1  = joints_vel[j, p]          # Current velocity      for v1 > v2
            v2  = joints_vel[j, p - 1]      # Previous velocity

        printm(f"wp1: {wp1}\twp2: {wp2}\tv1: {joints_vel[j, p - 1]} \tv2: {joints_vel[j, p]}")

        # Case: Speed change between positions
        if v1 < v2:
            # Enough distance for full acceleration ramp
            if dist > full_ramp_dist:
                # Use Sustained Acceleration Pulse
                vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2   # V_a
                vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2   # V_b
                dist_ramp_up    = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
                dist_cruise     = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
                dist_ramp_down  = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time
                
                if dist_ramp_up + dist_cruise + dist_ramp_down > dist:
                    assert 0, "Something went wrong, consider lowering v2"
                
                # Get values for quintic control points
                t0  = 0
                t1  = t0 + ramp_time
                t2  = t1 + (vel_before_ramp_down - vel_after_ramp_up) / max_acc
                t3  = t2 + ramp_time
                t4  = t3 + (dist - dist_ramp_up - dist_cruise - dist_ramp_down) / v2

                pos0 = wp1
                pos1 = pos0 + dist_ramp_up
                pos2 = pos1 + dist_cruise
                pos3 = pos2 + dist_ramp_down
                pos4 = wp2

                t   = [t0, t1, t2, t3, t4]
                pos = [pos0, pos1, pos2, pos3, pos4]
                vel = [v1, vel_after_ramp_up, vel_before_ramp_down, v2, v2]
                acc = [0, max_acc, max_acc, 0, 0]
                jrk = [0, 0, 0, 0, 0]

                if any(tt == np.nan or tt == np.inf for tt in t) or sum(t) == 0:
                    assert 0, "Invalid times: Something is wrong here. Check if position change makes sense with velocities."

                # Get quintic polynomial parameters
                p_params_pos = get_quintic_params(t, pos, vel, acc)
      
            else:
                # Use Acceleration Pulse
                assert 0, "not implemented yet"

        # Case: No speed change between positions
        if v1 == v2:
            t0  = 0
            t1  = dist / v1
            
            t   = [t0, t1]
            pos = [wp1, wp2]

            p_params_pos    = [[wp1, (wp2 - wp1) / (t1 - t0), 0, 0, 0]]
               
        # Mirror and shift polynoms if v2 < v1, set new offset times and mirror pos as well
        if joints_vel[j, p] < joints_vel[j, p - 1]:

            # Only for 4-segment quintic interpolation
            if len(p_params_pos) == 4:
                polycoefs_old   = p_params_pos.copy()
                times_old       = t.copy()
                pos_old         = pos.copy()
                durations       = [times_old[k + 1] - t[k] for k in range(4)]

                t[0] = 0
                for i in range(4):
                    old_poly        = Polynomial(polycoefs_old[i])
                    # Mirror along y axis and shift in x direction
                    temp_coefs      = old_poly.convert(window=[1 + durations[i], - 1 + durations[i]]).coef
                    # Mirror along x axis and shift in y direction
                    temp_coefs      = -temp_coefs
                    temp_coefs[0]   = temp_coefs[0] + (wp1 + (wp2 - wp1) / 2) * 2

                    p_params_pos[3 - i] = temp_coefs
                    t[i + 1]            = t[i] + durations[3 - i]

                for i in range(5):
                    pos[4 - i]  = -pos_old[i] + (wp1 + (wp2 - wp1) / 2) * 2
                    

        # Print outs and further calculations
        # Register trajectory segments
        for i, p_param_pos in enumerate(p_params_pos):
            if i == 0 or i == len(p_params_pos) : is_wp = True
            else: is_wp = False
            trajectory_joint0.register_segment(p_param_pos, t[i + 1] - t[i], pos[i], pos[i+1], is_wp)
            pass

        p_params_vel = [Polynomial(poly).deriv(1).coef for poly in p_params_pos]
        p_params_acc = [Polynomial(poly).deriv(2).coef for poly in p_params_pos]
        p_params_jrk = [Polynomial(poly).deriv(3).coef for poly in p_params_pos]
        # plot_trajectory_waypoint(p_params_pos, t, pos)
        # plot_trajectory_waypoint_limits(p_params_pos, p_params_vel, p_params_acc, p_params_jrk, t, pos, limits)
        

    plot_trajectory_stitched(trajectory_joint0)

    a = 1
    pass


pass