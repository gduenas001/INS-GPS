

classdef ParametersClass < handle
    
    properties (SetAccess = private)
        % --------------- Switches (options) ---------------
        SWITCH_REDUCE_TESTING= 0; % to test only a few frames 
        SWITCH_VIRT_UPDATE_Z= 0; % virtual update for the z-vel in the body frame
        SWITCH_VIRT_UPDATE_Y= 0; % virtual update for the y-vel in the body frame
        SWITCH_YAW_UPDATE= 1; 
        SWITCH_GPS_UPDATE= 1; % update of the GPS
        SWITCH_GPS_VEL_UPDATE= 1; % update of the GPS
        SWITCH_LIDAR_UPDATE= 1;
        SWITCH_REMOVE_FAR_FEATURES= 1;
        SWITCH_CALIBRATION= 1; % initial calibration to obtain moving biases
        % --------------------------------------------------
    end
    
    properties (Constant)
        
        % dataset obtained with ROS
%         path= '../data/cart/20180725/';
        path= '../data/vehicle/20190110/';     
        
        sn_f= ( 0.05 * 9.80279 / 1000 )^2 % bias acc white noise PSD
        sn_w= ( deg2rad(0.3/3600) )^2;    % bias gyro white noise PSD
    end
    
    properties (SetAccess = immutable) % parameters to be built with constructor
        
        file_name_imu
        file_name_gps
        file_name_lidar_path
        file_name_calibration

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
        
        num_epochs_incl_calibration
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

        VRW= 0.07 % vel random walk
        ARW= 0.15 % angular random walk [deg]
        
    end
        
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj = ParametersClass()
            
            % set file names
            obj.file_name_imu=  strcat(obj.path, 'IMU/IMU.mat');
            obj.file_name_gps=  strcat(obj.path, 'GPS/GPS.mat');
            obj.file_name_lidar_path= strcat(obj.path, 'LIDAR/');
            obj.file_name_calibration= strcat( obj.path, 'IMU/calibration.mat');
            
            %load dataset specific parameters
            run([obj.path, 'parameters.m']);
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
            
            % modify parameters
            obj.VRW= obj.VRW * obj.mult_factor_acc_imu; 
            obj.ARW= obj.ARW * obj.mult_factor_gyro_imu; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
            
            % build parameters
            obj.num_epochs_incl_calibration= round(obj.num_epochs_static);
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