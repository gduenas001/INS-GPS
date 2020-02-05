
classdef EstimatorClassEkfExpLinErr < handle
    properties (SetAccess = immutable)
        landmark_map
    end

    properties
        XX= zeros(15,1)
        x_true= zeros(3,1)
        alpha % state of interest extraction vector
        PX= zeros(15)
        PX_ub = zeros(15)


        association % association of current features
        association_full % association of current features
        association_true % only for simulation
        association_no_zeros % association of associated features
        num_landmarks % nunber of landmarks in the map
        num_associated_lms= 0
        num_extracted_features
        num_of_extracted_features
        number_of_associated_LMs

        n_k % number of absolute measurements at current time
        num_faults_k % number of injected faults at current time

        gamma_k
        q_k
        q_k_ub
        Y_k
        H_k
        L_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        Dc_cov_controller
        Dc_cov_state
        Y_k_ub
        L_k_ub
        D_ub

        T_d= 0 % detector threshold
        q_d= 0 % detector for the window of time

        initial_attitude % save initial attitude for the calibration of IM?U biases
        appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong
        FoV_landmarks_at_k % landmarks in the field of view
    end


    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClassEkfExpLinErr(imu_calibration_msmts, params)

            % Initial attitude
            obj.initialize_pitch_and_roll(imu_calibration_msmts)
            % initialize the yaw angle
            obj.XX(params.ind_yaw)= deg2rad(params.initial_yaw_angle);

            % save initial attitude for calibration
            obj.initial_attitude= obj.XX(7:9);

            % initialize covariance
            obj.PX= eps*eye(15);
            obj.PX(10:12, 10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
            obj.PX(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;

            % load map if exists
            data= load(strcat( params.path, 'landmark_map.mat' ));
            obj.landmark_map= data.landmark_map;
            obj.num_landmarks= size(obj.landmark_map, 1);
            obj.Dc_cov_state = compute_Decleene_cov(length(obj.XX));
            obj.Dc_cov_controller = compute_Decleene_cov(length(params.W_odometry_sim));
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
        increase_landmarks_cov(obj, minPXLM)
        % ----------------------------------------------
        % ----------------------------------------------
        addNewLM(obj, z, R)
        % ----------------------------------------------
        % ----------------------------------------------
        linearize_discretize(obj, u, dT, params, epoch)
        % ----------------------------------------------
        % ----------------------------------------------
        discretize(obj, F, G, S, Su, dT)
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_alpha(obj,params)
            obj.alpha= [-sin( obj.XX(params.ind_yaw) );...
                        cos( obj.XX(params.ind_yaw) );...
                       0 ];
        end
        % ----------------------------------------------
        % ----------------------------------------------
        Su = Hess_fn(obj, u, dT, params)
        % ----------------------------------------------
        % ----------------------------------------------
    end
    % ----------------------------------------------
    % ----------------------------------------------
end
