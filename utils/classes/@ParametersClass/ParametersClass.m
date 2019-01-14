

classdef ParametersClass < handle
    
    properties (SetAccess = private)
        % --------------- Switches (options) ---------------
        SWITCH_REDUCE_TESTING
        SWITCH_VIRT_UPDATE_Z
        SWITCH_VIRT_UPDATE_Y
        SWITCH_YAW_UPDATE
        SWITCH_GPS_UPDATE
        SWITCH_GPS_VEL_UPDATE
        SWITCH_LIDAR_UPDATE
        SWITCH_REMOVE_FAR_FEATURES
        SWITCH_CALIBRATION
        SWITCH_SLAM
        % --------------------------------------------------
    end
    
    properties (Constant)
        
%         % --------------- Parameters ---------------
%         min_appearances= 2; % only update estimate landmarks detected more than this number
%         num_epochs_reduce_testing= 5000;
%         num_epochs_static= 9700; % default (10000) --Osama-- Number of epochs the cart is static initially 20000
%         lidarRange= 25; % [m]
%         m_F= 2; % measurements per feature/landmark
%         dt_imu= 1/125; % IMU sampling time
%         dt_cal= 1/10; % KF Update period during initial calibration
%         dt_virt_z= 1/10; % Virtual msmt update period
%         dt_virt_y= 1/10; % Virtual msmt update period
%         sig_cal_pos= 0.01; % 3cm   -- do not reduce too much or bias get unstable
%         sig_cal_vel= 0.01; % 3cm/s -- do not reduce too much or bias get unstable
%         sig_cal_E= deg2rad(0.1); % 0.1 deg
%         sig_yaw0= deg2rad(5); % 5 deg -- Initial uncertatinty in attitude
%         sig_phi0= deg2rad(1); % 2 deg -- Initial uncertatinty in attitude
%         sig_ba= 0.05; % 0.1 m/s2 -- Initial acc bias uncertainty
%         sig_bw= deg2rad(0.1); % 0.2 deg/s -- Initial gyros bias uncertainty
%         sig_virt_vz= 0.01; % 5cm/s -- virtual msmt SD in z
%         sig_virt_vy= 0.01; % 5cm/s -- virtual msmt SD in y
%         sig_lidar= 0.25; % 20cm -- lidar measurement in the nav frame
%         min_vel_gps= 2/3.6; % 2 km/h
%         min_vel_yaw= 2/3.6; % 2 km/h
%         taua_normal_operation= 3000; % Tau for acc bias -- from manufacturer
%         tauw_normal_operation= 3000; % Tau for gyro bias -- from manufacturer
%         taua_calibration= 100; % 200 acc tau value during initial calibration
%         tauw_calibration= 100; % 200 gyro tau value during initial calibration
%         g_val= 9.80279; % value of g [m/s2] at the IIT
%         r_IMU2rearAxis= 0.9; % distance from IMU to rear axis
%         alpha_NN= 0.05; % prob of discard good features in NN
%         threshold_new_landmark= 15; % Threshold in NIS to create a new landmark
%         sig_minLM= 0.10; % minimum SD for the landmarks
%         mult_factor_acc_imu= 15; % multiplicative factor for the accel SD
%         mult_factor_gyro_imu= 10; % multiplicative factor for the gyros SD
%         mult_factor_pose_gps= 1; % multiplicative factor for the GPS pose SD
%         mult_factor_vel_gps= 1;  % multiplicative factor for the GPS velocity SD
%         feature_height= 1.5; % height of the features
%         initial_yaw_angle= -90 % [deg] initial yaw angle (different for each experiment) % smooth_turn(180)
%         preceding_horizon_size= 4;
%         continuity_requirement= 1e-5;
%         alert_limit= 1;
%         % -------------------------------------------
        
        % path= '../data/cart/20180725/';
        path= '../data/vehicle/20190110/';     
        
%         sn_f= ( 0.05 * 9.80279 / 1000 )^2 % bias acc white noise PSD
%         sn_w= ( deg2rad(0.3/3600) )^2;    % bias gyro white noise PSD
    end
    
    properties (SetAccess = immutable) % parameters to be built with constructor
        
        % ------------ paramters specified in file ----------------
        min_appearances
        num_epochs_reduce_testing
        num_epochs_static
        lidarRange
        m_F
        dt_imu
        dt_cal
        dt_virt_z
        dt_virt_y
        sig_cal_pos
        sig_cal_vel
        sig_cal_E
        sig_yaw0
        sig_phi0
        sig_ba
        sig_bw
        sig_virt_vz
        sig_virt_vy
        sig_lidar
        min_vel_gps
        min_vel_yaw
        taua_normal_operation
        tauw_normal_operation
        taua_calibration
        tauw_calibration
        g_val
        r_IMU2rearAxis
        alpha_NN
        threshold_new_landmark
        sig_minLM
        mult_factor_acc_imu
        mult_factor_gyro_imu
        mult_factor_pose_gps
        mult_factor_vel_gps
        feature_height
        initial_yaw_angle
        preceding_horizon_size
        continuity_requirement
        alert_limit
        VRW
        ARW
        sn_f
        sn_w
        % -------------------------------------------
        
        file_name_imu
        file_name_gps
        file_name_lidar_path
        file_name_calibration

        g_N % G estimation (sense is same at grav acceleration in nav-frame)
        sig_cal_pos_blkMAtrix
        sig_cal_vel_blkMAtrix
        sig_cal_E_blkMAtrix
        R_cal
        H_cal % Calibration observation matrix
        H_yaw
        R_virt_Z
        R_virt_Y
        R_lidar
        T_NN
        xPlot
        yPlot
        zPlot
        xyz_B
        R_minLM
        
        % IMU -- white noise specs
        sig_IMU_acc
        sig_IMU_gyr
        V
        Sv     % PSD for IMU
        Sv_cal % PSD during calibration
        
        % Biases -- PSD of white noise
        Sn_f % TODO: check if needed
        Sn_w % TODO: check if needed
        Sn
        
        % PSD for continuous model
        S
        S_cal
        
    end
        
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj = ParametersClass(navigation_type)
            
            
            % ------------ paramters specified in file ----------------
            run([obj.path, 'parameters.m']);
            
            % --------------- Switches (options) ---------------
            obj.SWITCH_REDUCE_TESTING= SWITCH_REDUCE_TESTING;
            obj.SWITCH_VIRT_UPDATE_Z= SWITCH_VIRT_UPDATE_Z;
            obj.SWITCH_VIRT_UPDATE_Y= SWITCH_VIRT_UPDATE_Y;
            obj.SWITCH_YAW_UPDATE= SWITCH_YAW_UPDATE;
            obj.SWITCH_GPS_UPDATE= SWITCH_GPS_UPDATE; 
            obj.SWITCH_GPS_VEL_UPDATE= SWITCH_GPS_VEL_UPDATE;
            obj.SWITCH_LIDAR_UPDATE= SWITCH_LIDAR_UPDATE;
            obj.SWITCH_REMOVE_FAR_FEATURES= SWITCH_REMOVE_FAR_FEATURES;
            obj.SWITCH_CALIBRATION= SWITCH_CALIBRATION;
             % --------------------------------------------------
            
            obj.min_appearances= min_appearances;
            obj.num_epochs_reduce_testing= num_epochs_reduce_testing;
            obj.num_epochs_static= num_epochs_static;
            obj.lidarRange= lidarRange;
            obj.m_F= m_F;
            obj.dt_imu= dt_imu;
            obj.dt_cal= dt_cal;
            obj.dt_virt_z= dt_virt_z;
            obj.dt_virt_y= dt_virt_y;
            obj.sig_cal_pos= sig_cal_pos;
            obj.sig_cal_vel= sig_cal_vel;
            obj.sig_cal_E= sig_cal_E;
            obj.sig_yaw0= sig_yaw0;
            obj.sig_phi0= sig_phi0;
            obj.sig_ba= sig_ba;
            obj.sig_bw= sig_bw;
            obj.sig_virt_vz= sig_virt_vz;
            obj.sig_virt_vy= sig_virt_vy;
            obj.sig_lidar= sig_lidar;
            obj.min_vel_gps= min_vel_gps;
            obj.min_vel_yaw= min_vel_yaw;
            obj.taua_normal_operation= taua_normal_operation;
            obj.tauw_normal_operation= tauw_normal_operation;
            obj.taua_calibration= taua_calibration;
            obj.tauw_calibration= tauw_calibration;
            obj.g_val= g_val;
            obj.r_IMU2rearAxis= r_IMU2rearAxis;
            obj.alpha_NN= alpha_NN;
            obj.threshold_new_landmark= threshold_new_landmark;
            obj.sig_minLM= sig_minLM;
            obj.mult_factor_acc_imu= mult_factor_acc_imu;
            obj.mult_factor_gyro_imu= mult_factor_gyro_imu;
            obj.mult_factor_pose_gps= mult_factor_pose_gps;
            obj.mult_factor_vel_gps= mult_factor_vel_gps;
            obj.feature_height= feature_height;
            obj.initial_yaw_angle= initial_yaw_angle;
            obj.preceding_horizon_size= preceding_horizon_size;
            obj.continuity_requirement= continuity_requirement;
            obj.alert_limit= alert_limit;
            obj.VRW= VRW;
            obj.ARW= ARW;
            obj.sn_f= sn_f;
            obj.sn_w= sn_w;
            % -------------------------------------------
            
            % differenciate between slam and localization
            if navigation_type == 'slam'
                obj.SWITCH_SLAM= 1;
            elseif navigation_type == 'localization'
                obj.SWITCH_SLAM= 0;
            else
                error('navigation_type must be either "slam" or "localization"');
            end
            
            % set file names
            obj.file_name_imu=  strcat(obj.path, 'IMU/IMU.mat');
            obj.file_name_gps=  strcat(obj.path, 'GPS/GPS.mat');
            obj.file_name_lidar_path= strcat(obj.path, 'LIDAR/');
            obj.file_name_calibration= strcat( obj.path, 'IMU/calibration.mat');

            
            % modify parameters
            obj.VRW= obj.VRW * obj.mult_factor_acc_imu; 
            obj.ARW= obj.ARW * obj.mult_factor_gyro_imu; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
            
            % build parameters
            obj.g_N= [0; 0; obj.g_val]; % G estimation (sense is same at grav acceleration in nav-frame)
            obj.sig_cal_pos_blkMAtrix= diag([obj.sig_cal_pos, obj.sig_cal_pos, obj.sig_cal_pos]);
            obj.sig_cal_vel_blkMAtrix= diag([obj.sig_cal_vel, obj.sig_cal_vel, obj.sig_cal_vel]);
            obj.sig_cal_E_blkMAtrix= diag([obj.sig_cal_E, obj.sig_cal_E, obj.sig_cal_E]);
            obj.R_cal= blkdiag(obj.sig_cal_pos_blkMAtrix, obj.sig_cal_vel_blkMAtrix, obj.sig_cal_E_blkMAtrix).^2;
            obj.H_cal= [eye(9), zeros(9,6)]; % Calibration observation matrix
            obj.H_yaw= [zeros(1,8),1,zeros(1,6)];
            obj.R_virt_Z= obj.sig_virt_vz.^2;
            obj.R_virt_Y= obj.sig_virt_vy.^2;
            obj.R_lidar= diag( [obj.sig_lidar, obj.sig_lidar] ).^2;
            obj.T_NN= 4.5; %chi2inv(1-obj.alpha_NN,2);
            xPlot= [-0.3; 0; -0.3];
            yPlot= [0.1; 0; -0.1];
            zPlot= [0; 0; 0];
            obj.xyz_B= [xPlot, yPlot, zPlot]';
            obj.R_minLM= obj.sig_minLM.^2;
            
            % IMU -- white noise specs
            obj.sig_IMU_acc= obj.VRW * sqrt( 2000 / 3600 );
            obj.sig_IMU_gyr= deg2rad( obj.ARW * sqrt( 2000 / 3600 ) ); % rad
            obj.V= diag( [obj.sig_IMU_acc; obj.sig_IMU_acc; obj.sig_IMU_acc;...
                      obj.sig_IMU_gyr; obj.sig_IMU_gyr; obj.sig_IMU_gyr]).^2;
            obj.Sv= obj.V * obj.dt_imu; % Convert to PSD
            obj.Sv_cal= diag( [diag( obj.Sv(1:3,1:3)) / obj.mult_factor_acc_imu^2; diag(obj.Sv(4:6,4:6)) / obj.mult_factor_gyro_imu^2]  );

            % Biases -- PSD of white noise
            obj.Sn_f= diag([obj.sn_f, obj.sn_f, obj.sn_f]);
            obj.Sn_w= diag([obj.sn_w, obj.sn_w, obj.sn_w]);
            obj.Sn= blkdiag(obj.Sn_f, obj.Sn_w);
            
            % PSD for continuous model
            obj.S     = blkdiag(obj.Sv, obj.Sn);
            obj.S_cal = blkdiag(obj.Sv_cal, obj.Sn);

        end
        % ----------------------------------------------
        % ----------------------------------------------
        function sig_yaw= sig_yaw_fn(~, v)
            sig_yaw= deg2rad(5) + ( exp(10*v)-1 )^(-1); %6.6035  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAREFUL
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function R_yaw= R_yaw_fn(obj, v)
            R_yaw= obj.sig_yaw_fn(v)^2;
        end 
        % ----------------------------------------------
        % ----------------------------------------------
        function turn_off_calibration(obj)
            obj.SWITCH_CALIBRATION= 0;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function turn_off_lidar(obj)
            obj.SWITCH_LIDAR_UPDATE= 0;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function turn_off_gps(obj)
            obj.SWITCH_GPS_UPDATE= 0;
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end 
end