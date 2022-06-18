import csv
import os
from re import X
from zlib import Z_FULL_FLUSH
import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial

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
waypoints   = get_list_from_csv("", "waypoints_01.csv")
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
            ax.plot(t, p, 'x', color='black', label='Control Point' if i == 1 else "")


    ax.set_xlim([-0.05, time_offset[-1] + 0.05])
    ax.set_ylim([min(pos) - 5, max(pos) + 5])
    ax.set_title('Quintic poly interpolation between two waypoints')
    ax.legend()
    plt.show()

def plot_trajectory_waypoint_limits(params_pos, params_vel, params_acc, params_jrk, time_offset, pos, limits, joint_id=0):

    fig, axs= plt.subplots(4, 1, sharex='col')

    x               = np.linspace(0, time_offset[-1], 1000)  # in seconds
    labels_segments = ['ramp up', 'cruise', 'ramp down', 'hold']
    labels_plots    = ['Position', 'Velocity', 'Acceleration', 'Jerk']
    poly_params     = [params_pos, params_vel, params_acc, params_jrk]
    max_value       = [limits['q_pos_max'][joint_id],  limits['q_vel_max'][joint_id],  limits['q_acc_max'][joint_id],  limits['q_jrk_max'][joint_id]]
    min_value       = [limits['q_pos_min'][joint_id], -limits['q_vel_max'][joint_id], -limits['q_acc_max'][joint_id], -limits['q_jrk_max'][joint_id]]

    for n in range(len(p_params_pos)):
        params  = poly_params[n]
        label_p = labels_plots[n]

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

        # Plot dots to show waypoints and limit yaxis
        if n == 0:
            axs[n].set_ylim([min(pos) - 5, max(pos) + 5])
            for i, (t, p) in enumerate(zip(time_offset, pos)):
                if i == 0 or i == len(time_offset) - 1:
                    axs[n].plot(t, p, 'o', color='black', label='Waypoints' if i == 0 else "")
                else:
                    axs[n].plot(t, p, 'x', color='black', label='Control Point' if i == 1 else "")

        axs[n].set_xlim([-0.05, time_offset[-1] + 0.05])
        axs[n].set_title(label_p)
    
    axs[len(axs) - 1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.5), ncol=4)
    fig.tight_layout()
    plt.show()

    pass


# Check if path has invalid values (out of min max bounds)
for j, joint_positions in enumerate(waypoints_t):
    for k, wp in enumerate(joint_positions):
        assert wp >= limits['q_pos_min'][j] and wp <= limits['q_pos_max'][j], f"Waypoint {k} of joint {j} out of q_min_pos, q_max_pos bounds: {wp}"

# Get trajectory parameters for joints--------------------------
joints_time = np.zeros(waypoints_t.shape)
joints_vel  = np.zeros(waypoints_t.shape)

# Iterate over joints
for j, joint_positions in enumerate(waypoints_t):   

    # Joint parameters
    max_vel     = limits['q_vel_max'][j]
    max_acc     = limits['q_acc_max'][j]
    max_jrk     = limits['q_jrk_max'][j]
    ramp_time   = (np.pi * max_acc) / (2 * max_jrk)    # Time for a full ramp with sinosodial acc profile
    joints_vel[j, 1:-1] = max_vel    # Set all but first and last speeds to max speed
  
    
    # Initial pos
    joints_time[j][0]  = 0
    
    # Iterate over positions
    for p in range(1, len(joint_positions)):        
        p1      = joint_positions[p - 1]    # Previous pos
        p2      = joint_positions[p]        # Current pos
        v1      = joints_vel[j, p - 1]      # Previous velocity
        v2      = joints_vel[j, p]          # Current velocity
        dist    = abs(p1 - p2)
        full_ramp_dist  = max_acc * (ramp_time ** 2) + 2 * joints_vel[j][p - 1] * ramp_time

        # Case: Speed change between positions
        if not v1 == v2:
            # Enough distance for full acceleration ramp
            if dist > full_ramp_dist:
                # Use Sustained Acceleration Pulse
                vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2
                vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2
                dist_ramp_up    = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
                dist_cruise     = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
                dist_ramp_down  = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time
                
                if dist_ramp_up + dist_cruise + dist_ramp_down > dist:
                    assert 0, "Something went wrong, consider lowering v2"
                
                # Get values for quintic control points /TODO /FIXME probably some mistake here
                t0  = 0
                t1  = t0 + ramp_time
                t2  = t1 + np.sqrt(2 * dist_cruise / max_acc)
                t3  = t2 + ramp_time
                t4  = t3 + (dist_ramp_up + dist_cruise + dist_ramp_down) / v2

                pos0 = p1
                pos1 = pos0 + dist_ramp_up
                pos2 = pos1 + dist_cruise
                pos3 = pos2 + dist_ramp_down
                pos4 = p2

                t   = [t0, t1, t2, t3, t4]
                pos = [pos0, pos1, pos2, pos3, pos4]
                vel = [v1, vel_after_ramp_up, vel_before_ramp_down, v2, v2]
                acc = [0, max_acc, max_acc, 0, 0]
                jrk = [0, 0, 0, 0, 0]

                # Get quintic polynomial parameters (also for vel, acc, jrk)
                p_params_pos = get_quintic_params(t, pos, vel, acc)
                p_params_vel = [Polynomial(poly).deriv(1).coef for poly in p_params_pos]
                p_params_acc = [Polynomial(poly).deriv(2).coef for poly in p_params_pos]
                p_params_jrk = [Polynomial(poly).deriv(3).coef for poly in p_params_pos]
                
                plot_trajectory_waypoint(p_params_pos, t, pos)
                plot_trajectory_waypoint_limits(p_params_pos, p_params_vel, p_params_acc, p_params_jrk, t, pos, limits)


            else:
                # Use Acceleration Pulse
                assert 0, "not implemented yet"

        # Case: No speed change between positions


pass