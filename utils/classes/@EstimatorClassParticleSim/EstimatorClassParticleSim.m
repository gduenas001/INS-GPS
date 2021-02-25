
classdef EstimatorClassParticleSim < handle
    properties (SetAccess = immutable)
        landmark_map
    end
    
    properties
        XX_prior= zeros(15,1)
        XX_predict= zeros(15,1)
        XX_update= zeros(15,1)
        x_true= zeros(3,1)
        alpha % state of interest extraction vector
        SX_prior= zeros(15)
        SX_predict= zeros(15)
        SX_update= zeros(15)
        
        
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
        Y_k
        H_k
        L_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        G_k_particles   % state evolution matrix
        F_k_particles % covariance increase for the state evolution
        
        
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
        detector_elapsed_time=0
        availability=0
        XX_particles_prior
        XX_particles_predict
        XX_particles_update
        Control_for_each_particle
        number_of_particles=200%4000
        number_of_particles_to_add=200
         m
        weight_vector
        %gen_estimate_cov=[0.2^2,0,0;0,0.2^2,0;0,0,deg2rad(10)^2]
        %gen_estimate_cov=[0.3^2,0,0;0,0.3^2,0;0,0,deg2rad(30)^2]
        %gen_estimate_cov=[0.3^2,0,0;0,0.3^2,0;0,0,deg2rad(25)^2]
        gen_estimate_cov=[0.1^2,0,0;0,0.1^2,0;0,0,deg2rad(10)^2]
        particles_indices_prior
        particles_indices_predict
        particles_indices_update
        g_k_bar
        h_k_i
        h_k
        z_k
        V_k
        %prior_estimate_cov
        H_k_particles
        threshold_add_particles=0.004%0.02
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClassParticleSim(params)
            
                % initialize sizes differently for simulation
                obj.m=3;
                obj.XX_prior= zeros(obj.m,1);
                obj.XX_prior(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                obj.XX_predict= zeros(obj.m,1);
                obj.XX_predict(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                obj.XX_update= zeros(obj.m,1);
                obj.XX_update(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                obj.x_true(params.ind_yaw)= deg2rad(params.initial_yaw_angle);
                % obj.PX= obj.gen_estimate_cov;
                obj.SX_prior= obj.gen_estimate_cov;
                obj.SX_predict= obj.gen_estimate_cov;
                obj.SX_update= obj.gen_estimate_cov;
                obj.XX_particles_prior = mvnrnd(obj.XX_predict,obj.SX_predict,obj.number_of_particles);
                obj.XX_particles_predict = obj.XX_particles_prior;
                obj.XX_particles_update = obj.XX_particles_prior;
                obj.particles_indices_prior = transpose( 1:size(obj.XX_particles_predict,1) );
                obj.particles_indices_predict = obj.particles_indices_prior;
                obj.particles_indices_update = obj.particles_indices_prior;
                for i=1:obj.number_of_particles
                    obj.XX_particles_prior(i,params.ind_yaw)= pi_to_pi(obj.XX_particles_prior(i,params.ind_yaw));
                    obj.XX_particles_predict(i,params.ind_yaw)= obj.XX_particles_prior(i,params.ind_yaw);
                    obj.XX_particles_update(i,params.ind_yaw)= obj.XX_particles_prior(i,params.ind_yaw);
                end
                obj.SX_prior= cov(obj.XX_particles_prior);
                obj.XX_prior=mean(obj.XX_particles_prior)';
                obj.SX_predict= obj.SX_prior;
                obj.XX_predict=obj.XX_prior;
                obj.SX_update= obj.SX_prior;
                obj.XX_update=obj.XX_prior;
                
                
                if params.SWITCH_GENERATE_RANDOM_MAP % map generated by params
                    obj.landmark_map= params.landmark_map;
                    obj.num_landmarks= size(obj.landmark_map, 1);
                else % map is loaded from saved variable
                    data= load(strcat( params.path, 'landmark_map.mat' ));
                    obj.landmark_map= data.landmark_map;
                    obj.num_landmarks= size(obj.landmark_map, 1);
                end
                        
        end
        % ----------------------------------------------
        % ----------------------------------------------
        odometry_update( obj, params )
        % ----------------------------------------------
        % ----------------------------------------------
        z= get_gps_msmt(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        gps_update(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        z= get_lidar_msmt(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        association= nearest_neighbor(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        lidar_update_localization(obj, z, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_steering(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        increase_landmarks_cov(obj, minPXLM)
        % ----------------------------------------------
        % ----------------------------------------------
        x= return_odometry_update(obj, x, u, params)
        % ----------------------------------------------
        % ----------------------------------------------
        [Phi, D_bar]= return_Phi_and_D_bar(obj, x, vel, phi, params)
        % ----------------------------------------------
        % ----------------------------------------------
        [G, F]= return_G_and_F(obj, x, vel, phi, params)
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_alpha(obj,params)
            
            obj.alpha= [-sin( obj.XX_update(params.ind_yaw) );...
                        cos( obj.XX_update(params.ind_yaw) );...
                       0 ];
                       
        end
        % ----------------------------------------------
        % ----------------------------------------------
        H_k = return_lidar_H(obj, x, association, params)
        % ----------------------------------------------
        % ----------------------------------------------
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



