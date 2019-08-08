

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
        SWITCH_FIXED_LM_SIZE_PH
        SWITCH_LM_SELECTION
        SWITCH_SEED
        SWITCH_ONLY_ONE_LM_FAULT
        SWITCH_GENERATE_RANDOM_MAP
        SWITCH_LIDAR_FAULTS
        SWITCH_GPS_FG
        SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt
        
        % switches set by the simulation name
        SWITCH_SLAM= 0
        SWITCH_SIM= 0
        SWITCH_FACTOR_GRAPHS= 0
        SWITCH_OFFLINE= 0
        % --------------------------------------------------
    end
    
    properties (Constant)
%         path_test= '../data/cart/20180725/';
        path_test= '../data/vehicle/20190110/';
        path_sim_kf= '../data/simulation/factor_graph/';
%         path_sim_kf= '../data/simulation/straight/';
%         path_sim= '../data/simulation/square/';
        path_sim_fg= '../data/simulation/factor_graph/';
        path_exp_fg= '../data/vehicle/20190110/';
    end
    
    properties (SetAccess = immutable) % parameters to be built with constructor
        
        % ---------------------------------------------------------
        % ------------ paramters specified in file ----------------
        % ---------------------------------------------------------
        m
        I_MA
        P_MA_max
        P_UA
        I_H
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
        M % same as preceding_horizon_size
        min_n_L_M
        continuity_requirement
        alert_limit
        VRW
        ARW
        sn_f
        sn_w
        
        % -------------------- simulation -----------------------
        num_epochs_sim
        dt_sim
        dt_gps
        velocity_sim
        steering_angle_sim
        sig_gps_sim
        sig_velocity_sim
        sig_steering_angle_sim
        R_gps_sim
        W_odometry_sim
        wheelbase_sim
        % -------------------------------------------
        % -------------------------------------------
        
        path
        file_name_imu
        file_name_gps
        file_name_lidar_path
        file_name_calibration
        
        ind_pose % indexes that extract x-y-theta
        ind_yaw % index that extracts theta

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
        sqrt_inv_R_lidar
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
        
        % ------------------ Factor Graphs ---------------------
        way_points
        min_distance_to_way_point
        max_delta_steering %maximum change in steering angle during one second
        max_steering
        velocity_FG
        FG_prec_hor
        sig_velocity_FG
        sig_steering_angle_FG
        W_odometry_FG
        wheelbase_FG
        min_state_var_FG
        sig_gyro_z
        map_limits % [x_min, x_max, y_min,  y_max]
        optimoptions % optimoptions for the fg optimization
        landmark_density % landmarks / m^2
        landmark_map
        
        % -------------------------------------------
        % -------------------------------------------
    end
        
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj = ParametersClass(navigation_type)
            
            % differenciate between slam and localization
            switch navigation_type 
                case 'slam'
                    obj.SWITCH_SLAM= 1;
                    obj.path= obj.path_test;
                case 'localization_kf'
                    obj.path= obj.path_test;
                case 'localization_fg'
                    obj.SWITCH_FACTOR_GRAPHS= 1;
                    obj.path= obj.path_test;
                case 'simulation_kf'
                    obj.SWITCH_SIM= 1;
                    obj.path= obj.path_sim_kf;
                case 'simulation_fg_offline'
                    obj.SWITCH_SIM= 1;
                    obj.SWITCH_FACTOR_GRAPHS= 1;
                    obj.SWITCH_OFFLINE= 1;
                    obj.path= obj.path_sim_fg;
                case 'simulation_fg_online'
                    obj.SWITCH_SIM= 1;
                    obj.SWITCH_FACTOR_GRAPHS= 1;
                    obj.path= obj.path_sim_fg;
                case 'experiment_fg_offline'
                    obj.SWITCH_SIM= 0;
                    obj.SWITCH_FACTOR_GRAPHS= 1;
                    obj.SWITCH_OFFLINE= 1;
                    obj.path= obj.path_exp_fg;
                otherwise
                    error('navigation_type must be either: "slam", "localization" or "factor_graph');
            end
                        
            % ---------------------------------------------------------
            % ------------ paramters specified in file ----------------
            % ---------------------------------------------------------
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
            obj.SWITCH_FIXED_LM_SIZE_PH= SWITCH_FIXED_LM_SIZE_PH;
            obj.SWITCH_LM_SELECTION= SWITCH_LM_SELECTION;
            obj.SWITCH_SEED= SWITCH_SEED;
            obj.SWITCH_ONLY_ONE_LM_FAULT= SWITCH_ONLY_ONE_LM_FAULT;
            
            if obj.SWITCH_FACTOR_GRAPHS
                obj.SWITCH_GENERATE_RANDOM_MAP= SWITCH_GENERATE_RANDOM_MAP;
                obj.SWITCH_LIDAR_FAULTS= SWITCH_LIDAR_FAULTS;
                if (~obj.SWITCH_SIM)
                    obj.SWITCH_GPS_FG= SWITCH_GPS_FG;
                    obj.SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt= SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt;
                end
            end
            
             % --------------------------------------------------
            obj.m= m;
            obj.I_MA= I_MA;
            obj.P_MA_max= P_MA_max;
            obj.P_UA= P_UA;
            obj.I_H= I_H;
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
            obj.M= preceding_horizon_size;
            obj.min_n_L_M= min_n_L_M;
            obj.continuity_requirement= continuity_requirement;
            obj.alert_limit= alert_limit;
            obj.VRW= VRW;
            obj.ARW= ARW;
            obj.sn_f= sn_f;
            obj.sn_w= sn_w;
            % -------------------- simulation -----------------------
            if obj.SWITCH_SIM
                obj.num_epochs_sim= num_epochs_sim;
                obj.dt_sim= dt_sim;
                obj.dt_gps= dt_gps;
                obj.velocity_sim= velocity_sim;
                obj.steering_angle_sim= steering_angle_sim;
                obj.sig_gps_sim= sig_gps_sim;
                obj.sig_velocity_sim= sig_velocity_sim;
                obj.sig_steering_angle_sim= sig_steering_angle_sim;
                obj.wheelbase_sim= wheelbase_sim;
                % if using factor graphs that needs controller
%                if obj.SWITCH_FACTOR_GRAPHS
                obj.way_points= way_points;
                obj.min_distance_to_way_point= min_distance_to_way_point;
                obj.max_delta_steering= max_delta_steering;
                obj.max_steering= max_steering;
                obj.sig_gyro_z= sig_gyro_z;
                obj.map_limits= map_limits;
                obj.landmark_density= landmark_density; 
%                end
            end
            % -------------------------------------------
            % -------------------------------------------
            
            % set file names
            obj.file_name_imu=  strcat(obj.path, 'IMU/IMU.mat');
            obj.file_name_gps=  strcat(obj.path, 'GPS/GPS.mat');
            obj.file_name_lidar_path= strcat(obj.path, 'LIDAR/');
            obj.file_name_calibration= strcat( obj.path, 'IMU/calibration.mat');

            
            % modify parameters
            obj.VRW= obj.VRW * obj.mult_factor_acc_imu; 
            obj.ARW= obj.ARW * obj.mult_factor_gyro_imu; %%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
            
            % ------------------ build parameters ------------------
            if obj.SWITCH_SEED
                obj.set_seed_to(SWITCH_SEED);
            else
%                 rng('shuffle')
            end
            
            if obj.SWITCH_SIM
                obj.ind_pose= 1:3;
                obj.ind_yaw= 3;
            else
                obj.ind_pose= [1,2,9];
                obj.ind_yaw= 9;
            end
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
            obj.sqrt_inv_R_lidar= sqrtm( inv( obj.R_lidar ) );
            obj.T_NN= chi2inv(1-obj.alpha_NN,2);
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
            
            % -------------------- simulation -----------------------
            obj.R_gps_sim= [obj.sig_gps_sim^2, 0; 
                            0, obj.sig_gps_sim^2];
                        
            obj.W_odometry_sim= [obj.sig_velocity_sim^2, 0;
                                 0, obj.sig_steering_angle_sim^2];
            % -------------------------------------------------------
            if obj.SWITCH_FACTOR_GRAPHS && ~obj.SWITCH_OFFLINE
                obj.optimoptions= optimoptions( @fminunc,...
                    'Display', optimoptions_display,...
                    'Algorithm','trust-region',...
                    'SpecifyObjectiveGradient', true,...
                    'HessianFcn', 'objective');
%                     'Algorithm','quasi-newton',...
            end
            
            % generate random map
            if obj.SWITCH_GENERATE_RANDOM_MAP
                obj.landmark_map= obj.return_random_map();
            end
            
            % Debugging examples of landmark maps
            % obj.landmark_map= [-10.9771207481359,2.45300831544226;6.82608325798797,-73.1977929704449;89.5254275520182,-7.25453088645978;69.3398428399569,-48.8455972810412;114.138015445711,7.59971722452418;-6.44463501373658,-25.3416514918823;13.9192416318387,-14.3256046082598;-7.94271327641291,-7.48138926822496;29.6940122816621,37.7109503452478;14.1645971451474,29.7711855572875;74.3102246290761,17.3527677747590;-5.51789508821601,45.3269050389306;-6.49529972209515,65.4399001956805;99.6596174159157,-39.2371589565058;40.0752853981384,3.60822696762624;89.9944937044295,69.2199345817613;-18.4833642650074,-0.955765466280809;11.0461954904917,-20.7381963865714;91.7682610078083,52.8560314670443;87.0765301960277,22.0728675799338;-4.48118755183638,-65.9624870567692;2.09681884055983,23.8958079086734;-22.0400992168105,-61.1069502958662];
            % obj.landmark_map= [-19.2774312209662,40.7797185222247;59.5705993506951,68.4223427974766;117.850869342007,49.6705256737122;60.1459688918045,-4.29842354000353;101.819694216991,13.8251422010301;-12.5688746372810,10.3136104813153;-13.9277636518960,-20.5550507384034;-22.7018267455224,-6.84870330092294;51.3962526676511,-39.2466178828560;96.4565013671369,20.4352206248171;65.5377942259571,-47.5557574518067;32.5792016371773,-74.3808199744421;73.4539646924914,2.74001094196981;89.0013294523708,18.5805534077841;27.4379300947862,-20.3596935284201;66.5356210683029,48.5736256997634;9.57781595754895,71.3038070336353;83.0186438364574,-21.1886649971006;111.193736810766,-56.4883691287973;95.8978125038271,1.75304139073606;97.2231654030829,70.5468329863530;107.770973263735,65.2853361241904;44.8379555623654,38.0575870723817];
            % obj.landmark_map= [114.561090901347,-8.69890651128443;90.8910816769900,-20.0852116676695;14.5249429017169,20.7628311222522;58.7228710109234,39.0004333523196;-11.9106913982250,41.6141419492330;41.7466207484293,72.5575699326742;63.8506034219044,58.2471531789660;29.2369687285187,28.6845628916225;-21.9914328868202,-2.27876926286363;-16.2108849485674,-74.2923774948769;93.0934074006759,-46.5966542168596;42.6292471795551,-32.8873910597020;117.790027351616,-62.8566070631853;63.4072758103570,-64.7664208004892;65.6329021201969,-58.3640409508784;77.7717289025263,54.5452155542394;76.9083075970530,-49.8121574402523;19.5961218218142,-37.7862572011971;-22.5066560623651,-13.3957625685328;-15.8566214611981,-55.5192929512514;80.9235475291791,-41.6523588039393;101.116353164894,27.6850395193426;111.060337381971,-18.2956427166910];
            
        end
        
        
        
        % ----------------------------------------------
        % ----------------------------------------------
        function landmark_map= return_random_map(obj)
            x_dim= obj.map_limits(2) - obj.map_limits(1);
            y_dim= obj.map_limits(4) - obj.map_limits(3);
            map_area= x_dim * y_dim;
            num_landmarks= round(obj.landmark_density * map_area);
            landmark_map= [rand( num_landmarks, 1 ) .* x_dim + obj.map_limits(1),...
                           rand( num_landmarks, 1 ) .* y_dim + obj.map_limits(3)];
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function sig_yaw= sig_yaw_fn(~, v)
            sig_yaw= deg2rad(5) + ( exp(10*v)-1 )^(-1); %6.6035  %%%%%%%%%%%%%%%%%%%%%%% CAREFUL
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
        function set_seed_to(obj, seed)
            rng(seed);
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end 
end