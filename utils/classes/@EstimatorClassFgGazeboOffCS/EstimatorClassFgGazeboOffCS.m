
classdef EstimatorClassFgGazeboOffCS < handle
    properties (SetAccess = immutable)
        landmark_map
    end
    
    properties
        XX= zeros(9,1) %Osama (15,1)
        x_true= zeros(3,1)
        alpha % state of interest extraction vector
        PX= eye(9)*eps %Osama (15)
        
        association % association of current features
        num_landmarks % number of landmarks in the map
        
        n_k % number of absolute measurements at current time
        
        q_k
        H_k
        Phi_k   % state evolution matrix
        D_bar % covariance increase for the state evolution
        
        T_d= 0 % detector threshold
        q_d= 0 % detector for the window of time
        
        FoV_landmarks_at_k % landmarks in the field of view
        lm_ind_fov % indexes of the landmarks in the field of view
        
        M= 0 % preceding horizon size in epochs
        PX_prior % cov matrix of the prior
        Gamma_prior % information matrix of the prior
        m_M % number of states to estimate
        n_total % total numbe of msmts
        x_prior % stores x_{k-M} as a msmt for the next epoch
        n_L_k= 0 % number of associations at k
        n_L_M= 0 % number of associations in the ph
        H_k_gps
        H_k_lidar
        n_gps_k=0
        n_L_k_ph % number of associations in the ph
        GPS_L_M=0
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= EstimatorClassFgGazeboOffCS(params)
            
            % initialize preceding horizon size
            if params.SWITCH_FIXED_LM_SIZE_PH
                obj.M= 0;
            else
                obj.M= params.M;
            end
            
            % initialize to uninformative prior
            obj.PX_prior= diag( ones(9,1) * eps ); %Osama params.m
            % initialize covariance
            %obj.PX_prior(10:12, 10:12)= diag([params.sig_ba,params.sig_ba,params.sig_ba] ).^2; %Osama
            %obj.PX_prior(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2; %Osama
            obj.Gamma_prior= inv(obj.PX_prior);
            obj.x_prior= zeros(9, 1); %Osama params.m
            % allocate memory
            obj.n_L_k_ph= zeros(params.M, 1);

            
            
            % load map if exists
            data= load(strcat( params.path, 'landmark_map.mat' ));
            obj.landmark_map= data.landmark_map;
            obj.num_landmarks= size(obj.landmark_map, 1);
                        
        end
        % ----------------------------------------------
        % ----------------------------------------------
        linearize_discretize(obj, u, dT, params)
        % ----------------------------------------------
        % ----------------------------------------------
        discretize(obj, F, G, S, dT)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_lidar_H_k(obj, params, FG, epoch)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_gps_H_k(obj, params, FG, epoch)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_imu_Phi_k(obj, params, FG, epoch)
        % ----------------------------------------------
        % ----------------------------------------------
        [z,association]= nearest_neighbor(obj, z, params)
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_alpha(obj,params)
            obj.alpha= [-sin( obj.XX(params.ind_yaw) );...
                        cos( obj.XX(params.ind_yaw) );...
                       zeros(7,1) ]; %Osama (13,1)
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
    % ----------------------------------------------
    % ----------------------------------------------
end



