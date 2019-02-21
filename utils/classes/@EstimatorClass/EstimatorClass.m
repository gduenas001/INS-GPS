
classdef EstimatorClass < handle
    properties (SetAccess = immutable)
        landmark_map
    end
    
    properties
        XX= zeros(15,1)
        x_true= zeros(3,1)
        
        PX= zeros(15)
        
        association % association of current features
        association_full % association of current features
        association_true % only for simulation
        association_no_zeros % association of associated features
        number_of_associated_LMs= 0
        num_of_extracted_features
        
        n_k % number of absolute measurements at current time
        n_L_k % number of landmarks extracted at current time

        gamma_k
        q_k
        Y_k
        H_k
        L_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        
        
        num_landmarks % nunber of landmarks in the map
        
        
        initial_attitude % save initial attitude for the calibration of IM?U biases
        appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong
        FoV_landmarks_at_k % landmarks in the field of view
        current_wp_ind= 1
        goal_is_reached= 0
        steering_angle= 0
        lm_ind_fov % indexes of the landmarks in the field of view
        
        x_ph % poses in the time window
        z_fg % all the msmts in the time window
        z_lidar_ph % lidar msmts in the preceding horizon
        z_lidar
        z_gyro= 0 % current gyro msmt
        z_gyro_ph % gyro msmts in the ph
        PX_prior % cov matrix of the prior
        Gamma_prior % information matrix of the prior
        m_M % number of states to estimate
        n_total % total numbe of msmts
        association_ph % associations during the ph
        odometry_k % odometry msmts at the current time
        odometry_ph % velocity and steering angle for the ph
        x_prior % stores x_{k-M} as a msmt for the next epoch
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClass(imu_calibration_msmts, params)
            
            if params.SWITCH_SIM
                % initialize sizes differently for simulation
                obj.XX= zeros(3,1);
                obj.XX(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                obj.x_true(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                obj.PX= eye(3) * eps;
                
            else
                % Initial attitude
                obj.initialize_pitch_and_roll(imu_calibration_msmts)
                % initialize the yaw angle
                obj.XX(params.ind_yaw)= deg2rad(params.initial_yaw_angle);

                % save initial attitude for calibration
                obj.initial_attitude= obj.XX(7:9);
                
                % initialize covariance
                obj.PX(10:12, 10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
                obj.PX(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
            end
            
            
            if params.SWITCH_FACTOR_GRAPHS
                % initilize the prior to uninformative
                obj.PX_prior= diag( ones(params.m,1) * 1000 );
                obj.Gamma_prior= zeros(params.m);
                
                % allocate memory
                obj.x_ph= cell(1, params.M);
                obj.z_lidar_ph= cell(1, params.M);
                obj.association_ph= cell(1, params.M);
                obj.odometry_ph= cell(1, params.M);
                obj.z_gyro_ph= cell(1, params.M);
            end
            
            
            % load map if exists
            if params.SWITCH_SLAM 
                obj.num_landmarks= 0;
            else % in localization map is knonw a priori
                data= load(strcat( params.path, 'landmark_map.mat' ));
                obj.landmark_map= data.landmark_map;
                obj.num_landmarks= size(obj.landmark_map, 1);
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        initialize_pitch_and_roll(obj, imu_calibration_msmts)
        % ----------------------------------------------
        % ----------------------------------------------     
        calibration(obj, imu_msmt, params)
        % ----------------------------------------------
        % ----------------------------------------------
        imu_update( obj, imu_msmt, params )
        % ----------------------------------------------
        % ----------------------------------------------
        odometry_update_sim( obj, imu_msmt, params )
        % ----------------------------------------------
        % ----------------------------------------------
        yaw_update(obj, w, params)
        % ----------------------------------------------
        % ----------------------------------------------
        yaw= yawMeasurement(obj, w, params)
        % ----------------------------------------------
        % ----------------------------------------------
        vel_update_z(obj, R)
        % ----------------------------------------------
        % ----------------------------------------------
        z= get_gps_msmt_sim(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        gps_update(obj, z, R, params)
        % ----------------------------------------------
        % ----------------------------------------------
        gps_update_sim(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        z= get_lidar_msmt_sim(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        association= nearest_neighbor_slam(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        association= nearest_neighbor_localization(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        lidar_update_slam(obj, z, association, params)
        % ----------------------------------------------
        % ----------------------------------------------     
        lidar_update_localization(obj, z, association, params)
        % ----------------------------------------------
        % ----------------------------------------------     
        lidar_update_localization_sim(obj, z, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_steering(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_lidar_jacobian_k(obj, params)
        % ----------------------------------------------
        % ---------------------------------------------- 
        update_z_fg(obj, counters, params)
        % ----------------------------------------------
        % ---------------------------------------------- 
        return_lidar_A(obj, x, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
        [Phi, D_bar]= return_Phi_and_D_bar(obj, x, vel, phi, params)
        % ----------------------------------------------
        % ----------------------------------------------
        solve_fg(obj, counters, params)
        % ----------------------------------------------
        % ----------------------------------------------
        x= return_odometry_update_sim(obj, x, u, params)
        % ----------------------------------------------
        % ----------------------------------------------
        z_gyro= generate_gyro_msmt_sim(obj, theta_prev, theta_next, params)
        % ----------------------------------------------
        % ----------------------------------------------
        z_expected= return_expected_z_lidar(obj, x, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
        A= return_A_fg(obj, x, params)
        % ----------------------------------------------
        % ----------------------------------------------
        [cost]= optimization_fn_fg(obj, x, params)
        % ----------------------------------------------
        % ----------------------------------------------
        x= from_estimator_to_vector(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        from_vector_to_estimator(obj, x, params)
        % ----------------------------------------------
        % ----------------------------------------------
        update_preceding_horizon(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        increase_landmarks_cov(obj, minPXLM)
        % ----------------------------------------------
        % ----------------------------------------------
        addNewLM(obj, z, R)
        % ----------------------------------------------
        % ----------------------------------------------
        linearize_discretize(obj, u, dT, params)
        % ----------------------------------------------
        % ----------------------------------------------
        discretize(obj, F, G, S, dT)
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



