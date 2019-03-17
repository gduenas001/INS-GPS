
classdef EstimatorClassSlam < handle
    properties (SetAccess = immutable)
        landmark_map
    end
    
    properties
        XX= zeros(15,1)
        x_true= zeros(3,1)
        alpha % state of interest extraction vector
        PX= zeros(15)
        
        
        association % association of current features
        association_full % association of current features
        association_true % only for simulation
        association_no_zeros % association of associated features
        num_landmarks= 0 % nunber of landmarks in the map
        num_associated_lms= 0
        num_extracted_features
        num_of_extracted_features
        number_of_associated_LMs
        
        n_k % number of absolute measurements at current time
        num_faults_k % number of injected faults at current time

        gamma_k
        q_k
        Y_k
        H_k
        L_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        
        T_d= 0 % detector threshold
        q_d= 0 % detector for the window of time
        
        initial_attitude % save initial attitude for the calibration of IM?U biases
        appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong
        FoV_landmarks_at_k % landmarks in the field of view
        current_wp_ind= 1 % index of the sought way point
        goal_is_reached= 0
        steering_angle= 0
        lm_ind_fov % indexes of the landmarks in the field of view
        
        M= 0 % preceding horizon size in epochs
        x_ph % poses in the time window
        z_fg % all the msmts in the time window
        z_lidar_ph % lidar msmts in the ph
        z_lidar % current lidar msmts
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
        n_L_k= 0 % number of associations at k
        n_L_M= 0 % number of associations in the ph
        H_k_gps
        H_k_lidar
        n_gps_k
        n_L_k_ph % number of associations in the ph
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClassSlam(imu_calibration_msmts, params)
            
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
        yaw_update(obj, w, params)
        % ----------------------------------------------
        % ----------------------------------------------
        yaw= yawMeasurement(obj, w, params)
        % ----------------------------------------------
        % ----------------------------------------------
        vel_update_z(obj, R)
        % ----------------------------------------------
        % ----------------------------------------------
        gps_update(obj, z, R, params)
        % ----------------------------------------------
        % ----------------------------------------------
        association= nearest_neighbor(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        lidar_update(obj, z, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
        addNewLM(obj, z, R)
        % ----------------------------------------------
        % ----------------------------------------------
        increase_landmarks_cov(obj, minPXLM)
        % ----------------------------------------------
        % ----------------------------------------------
        linearize_discretize(obj, u, dT, params)
        % ----------------------------------------------
        % ----------------------------------------------
        discretize(obj, F, G, S, dT)
        % ----------------------------------------------
        % ----------------------------------------------
        
        function compute_alpha(obj,params)

            obj.alpha= [-sin( obj.XX(params.ind_yaw) );...
                        cos( obj.XX(params.ind_yaw) );...
                       0 ];

        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



