def OBS_get_waypoint_parameters_v2(self, waypoints_t : np.array):
    ''' This is the old version which tries to 
    
        To synchronise all joints, they must be at a waypoint at the same time. 
        To achieve this, the target velocity for each joint at each waypoint is computed.
        
        1.  Depending on the joint positions, initial velocities (max_vel, 0, -max_vel) 
            are set at each waypoint. 
                wp1 < wp2 : v1 = max_vel
                wp1 > wp2 : v1 = -max_vel
                First and last v are set to 0

        For each waypoint
            For each joint

        2.  The duration is computed which is needed to transition to the next waypoint
        3.  The max. duration of all joints is set for all joints, this way the slowest
            joint will have its min duration while all other joints are slowed down
        4.  The corresponding velocity is computed, such that the desired delay is achieved.
            This is done numerically.           
        
        Parameters:
            waypoints_t (List): Waypoint positions for joints [joint_id][wp]
        Returns:
            velocities_t (List): Joint velocities corresponding to each joint and waypoint [joint_id][wp]
    '''
    initial_vel = np.zeros(waypoints_t.shape)       # [joint_id][wp] 
    initial_dur = np.zeros(waypoints_t.shape[0])    # [joint_id]
    synced_dur  = 0                                 # Scalar
    target_vel  = np.zeros(waypoints_t.shape)       # Target velocities in [angle/s]

    num_joints  = waypoints_t.shape[0]
    num_wp      = waypoints_t.shape[1]


    # 1. Generate initial joint velocities at waypoints
    for joint_id, joint_wp in enumerate(waypoints_t):
        initial_vel[joint_id, 0]    = 0             # First vel is always 0

        for wp_id in range(1 , num_wp - 1):
            wpp     = waypoints_t[joint_id, wp_id - 1]
            wp1     = waypoints_t[joint_id, wp_id]
            wp2     = waypoints_t[joint_id, wp_id + 1]
            
            if   wpp < wp1 < wp2: vel_f =  1
            elif wpp > wp1 > wp2: vel_f = -1
            else:                 vel_f =  0

            initial_vel[joint_id, wp_id]  = vel_f * self.qlimit_vel_max[joint_id]

        initial_vel[joint_id, -1]   = 0             # Last vel is always 0


    # Iterate over waypoints
    for wp_id in range(num_wp - 1):

        # /TODO: Integrate 1. here

        # 2. Get initial durations for each waypoint transition
        for joint_id in range(num_joints):
            wp1     = waypoints_t[joint_id, wp_id]
            wp2     = waypoints_t[joint_id, wp_id + 1]
            v1      = initial_vel[joint_id, wp_id]
            v2      = initial_vel[joint_id, wp_id + 1]
            max_vel = self.qlimit_vel_max[joint_id]
            max_acc = self.qlimit_acc_max[joint_id]
            max_jrk = self.qlimit_jrk_max[joint_id]
            
            initial_dur[joint_id]   = self.get_4segments_duration(wp1, wp2, v1, v2, max_vel, max_acc, max_jrk)

        # 3. Synchronise duration
        synced_dur = max(initial_dur)
        print(f"synced_dur: {synced_dur} for wp {wp_id}")

        # 4. Get synchronized velocities at waypoints
        for joint_id in range(num_joints):
            wp1     = waypoints_t[joint_id, wp_id]
            wp2     = waypoints_t[joint_id, wp_id + 1]
            v1      = target_vel[joint_id, wp_id]
            max_vel = self.qlimit_vel_max[joint_id]
            max_acc = self.qlimit_acc_max[joint_id]
            max_jrk = self.qlimit_jrk_max[joint_id]

            target_vel[joint_id, wp_id + 1], _ = self.get_4segments_v2(synced_dur, wp1, wp2, v1, max_vel, max_acc, max_jrk)
    
    target_vel[:,-1] = 0

    return target_vel

# Old get trajectory, based on variable v2
def get_trajectory(self, waypoints_t : np.array, velocities_t : np.array):
    ''' Returns a trajectory object for given
        Parameters:
            waypoints (List): Waypoint positions for joints [joint_id][pos]
            '''

    # Check if path has invalid values (out of min max bounds)
    for joint_id, joint_pos in enumerate(waypoints_t):
        for k, wp in enumerate(joint_pos):
            assert wp >= limits['q_pos_min'][joint_id] and wp <= limits['q_pos_max'][joint_id], f"Waypoint {k} of joint {joint_id} out of q_min_pos, q_max_pos bounds: {wp}"

    """ Notes:
        - No two consecutive waypoints can have 0 velocity
        - No two consecutive waypoints can have the same pos
        - Implicit 0 velocity crossing is possible, however, pos bound is not guaranteed
    """
    # TODO Check if velocities comply with notes above
    # TODO Make safety check at the end for pos as well

    # Create trajectory objects
    for joint_id in range(waypoints_t.shape[0]):
        self.trajectories.append(Trajectory(joint_id, 0))

    # Iterate over joints
    for joint_id, joint_pos in enumerate(waypoints_t):

        # Joint parameters
        max_vel     = self.qlimit_vel_max[joint_id]
        max_acc     = self.qlimit_acc_max[joint_id]
        max_jrk     = self.qlimit_jrk_max[joint_id]
        ramp_time   = (np.pi * max_acc) / (2 * max_jrk)    # Time for a full ramp with sinosodial acc profile

        trajectory  = self.trajectories[joint_id]

        # Iterate over positions
        for p in range(1, len(joint_pos)):

            # Setup------------------------------------------------------------------------------------
            og_wp1  = joint_pos[p - 1]      # Previous waypoint
            og_wp2  = joint_pos[p]          # Current waypoint
            wp_increase = og_wp1 < og_wp2   # Flag for swapped waypoints (in case og_wp1 > og_wp2)

            og_v1   = velocities_t[joint_id][p - 1]     # Previous velocity
            og_v2   = velocities_t[joint_id, p]         # Current velocity
            v_increase  = abs(og_v1) < abs(og_v2)       # Flag for swapped velocities (in case og_v1 > og_v2)

            dist    = abs(og_wp1 - og_wp2)
            full_ramp_dist  = max_acc * (ramp_time ** 2) + 2 * og_v1 * ramp_time

            # Mirroring to fit standard interpolation pattern
            # Normal case
            if wp_increase and v_increase:
                wp1 = og_wp1
                wp2 = og_wp2
                v1  = og_v1
                v2  = og_v2
            # Swap velocities, so we always have a rising velocity for calculations
            if wp_increase and not v_increase:
                wp1 = og_wp1
                wp2 = og_wp2
                v1  = og_v2
                v2  = og_v1
            # Swap waypoints, so we always have rising position for calculation
            if not wp_increase and v_increase:
                wp1 = og_wp2
                wp2 = og_wp1
                v1  = - og_v1
                v2  = - og_v2
            # Swap waypoints and velocities
            if not wp_increase and not v_increase:
                wp1 = og_wp2
                wp2 = og_wp1
                v1  = - og_v2
                v2  = - og_v1
            
            printm(f"Original: wp1: {og_wp1:.2f}\twp2: {og_wp2:.2f}\tv1: {og_v1:.2f} \tv2: {og_v2:.2f}")
            printm(f"Modified: wp1: {wp1:.2f}\twp2: {wp2:.2f}\tv1: {v1:.2f} \tv2: {v2:.2f}")
            printm(f"wp_increase: {wp_increase}, v_increase: {v_increase}")

            # Get and register trajectory segment------------------------------------------------------
            # Case: Speed change between positions
            if v1 < v2:
                # Make sure that there is enough distance for full acceleration ramp
                if not dist > full_ramp_dist or not v2 - v1 > ramp_time * max_acc:
                    assert 0, "Use Acceleration Pulse, or make dist larger - not implemented yet"

                # Loop in case v2 is too large
                while True:
                    # Use Sustained Acceleration Pulse
                    vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2   # V_a
                    vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2   # V_b
                    
                    dist_ramp_up    = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
                    dist_cruise     = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
                    dist_ramp_down  = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time
                    
                    if dist_ramp_up + dist_cruise + dist_ramp_down > dist: 
                        printm(f"Warning: v2 too large, reducing v2 from {v2:.4f} to {v2 * 0.9:.4f}")
                        v2 = v2 * 0.9       # TODO: this must be given back and stored in velocities_t
                        assert v2 >= 0.001, "Something went wrong, consider lowering v2 or increasing distance"
                    else:
                        break
                
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

                if any(tt == np.nan or tt == np.inf for tt in t) or sum(t) == 0:
                    assert 0, "Invalid times: Something is wrong here. Check if position change makes sense with velocities."

                poly_coef   = self.__get_quintic_params(t, pos, vel, acc) 

            # Case: No speed change between positions
            elif v1 == v2:
                t0  = 0
                t1  = dist / v1
                
                t   = [t0, t1]
                pos = [wp1, wp2]

                poly_coef   = [[wp1, (wp2 - wp1) / (t1 - t0), 0, 0, 0]]

            # Catch errror
            else:
                assert 0, 'No polyonom has been computed, check velocities'

            # Mirror and shift polynoms, set new offset times and mirror pos as well
            if not v_increase:

                l               = len(poly_coef)
                polycoefs_old   = poly_coef.copy()
                times_old       = t.copy()
                pos_old         = pos.copy()
                durations       = [times_old[k + 1] - t[k] for k in range(l)]

                t[0] = 0
                for i in range(l):
                    old_poly        = Polynomial(polycoefs_old[i])
                    # Mirror along y axis and shift in x direction
                    temp_coefs      = old_poly.convert(window=[1 + durations[i], - 1 + durations[i]]).coef
                    # Mirror along x axis and shift in y direction
                    temp_coefs      = -temp_coefs
                    temp_coefs[0]   = temp_coefs[0] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                    poly_coef[l - 1 - i] = temp_coefs
                    t[i + 1]            = t[i] + durations[l - 1 - i]

                for i in range(l + 1):
                    pos[l - i]  = -pos_old[i] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

            # Mirror over offsetted x, set new pos as well
            if not wp_increase :

                l               = len(poly_coef)
                polycoefs_old   = poly_coef.copy()
                pos_old         = pos.copy()

                for i in range(l):
                    old_poly        = Polynomial(polycoefs_old[i])
                    temp_coefs      = old_poly.coef
                    # Mirror along x axis and shift in y direction
                    temp_coefs      = -temp_coefs
                    temp_coefs[0]   = temp_coefs[0] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                    poly_coef[i] = temp_coefs

                for i in range(l + 1):
                    pos[i]  = - pos_old[i] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

            # Register trajectory segments
            for i, p_param_pos in enumerate(poly_coef):
                is_wp = (i == 0 or i == len(poly_coef))
                trajectory.register_segment(p_param_pos, t[i + 1] - t[i], pos[i], pos[i+1], is_wp)

            #plot_trajectory_waypoint(poly_coef, t, pos)
            #plot_trajectory_waypoint_limits(poly_coef, t, pos, limits)

        plot_trajectory_stitched(trajectory)
        print(t)
        a = 1
        pass
    pass