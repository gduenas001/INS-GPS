

classdef ParametersClass
    properties (Constant)
        % --------------- Switches (options) ---------------
        SWITCH_NUM_of_LOOPS= 1; % --Osama--
        SWITCH_CALIBRATION= 1; % initial calibration to obtain moving biases
        SWITCH_VIRT_UPDATE_Z= 0; % virtual update for the z-vel in the body frame
        SWITCH_VIRT_UPDATE_Y= 0; % virtual update for the y-vel in the body frame
        SWITCH_YAW_UPDATE= 1;
        SWITCH_GPS_UPDATE= 1; % update of the GPS
        SWITCH_GPS_VEL_UPDATE= 1; % update of the GPS
        SWITCH_LIDAR_UPDATE= 1;
        SWITCH_REMOVE_FAR_FEATURES= 1;
        % --------------------------------------------------
        
        % --------------- Parameters ---------------
        dT_IMU= 1/125; % IMU sampling time
        dT_cal= 1/10; % KF Update period during initial calibration
        dT_virt_Z= 1/10; % Virtual msmt update period
        dT_virt_Y= 1/10; % Virtual msmt update period
        numEpochStatic= 20000; % default (10000) --Osama-- Number of epochs the cart is static initially 20000
        sig_cal_pos= 0.005; % 3cm   -- do not reduce too much or bias get instable
        sig_cal_vel= 0.005; % 3cm/s -- do not reduce too much or bias get instable
        sig_cal_E= deg2rad(0.1); % 0.1 deg
        sig_yaw0= deg2rad(5); % 5 deg -- Initial uncertatinty in attitude
        sig_phi0= deg2rad(1); % 2 deg -- Initial uncertatinty in attitude
        sig_ba= 0.05; % 0.1 m/s2 -- Initial acc bias uncertainty
        sig_bw= deg2rad(0.1); % 0.2 deg/s -- Initial gyros bias uncertainty
        sig_virt_vz= 0.01; % 5cm/s -- virtual msmt SD in z
        sig_virt_vy= 0.01; % 5cm/s -- virtual msmt SD in y
        sig_lidar= 0.3; % 20cm -- lidar measurement in the nav frame
        sig_yaw_fn= @(v) deg2rad(5) + ( exp(10*v)-1 )^(-1); %6.6035  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAREFUL
        minVelocityGPS= 2/3.6; % 2 km/h
        minVelocityYaw= 2/3.6; % 2 km/h
        taua0= 3000; % Tau for acc bias -- from manufacturer
        tauw0= 3000; % Tau for gyro bias -- from manufacturer
        taua_calibration= 100; % 200 acc tau value during initial calibration
        tauw_calibration= 100; % 200 gyro tau value during initial calibration
        g_val= 9.80279; % value of g [m/s2] at the IIT
        r_IMU2rearAxis= 0.9; % distance from IMU to rear axis
        lidarRange= 25; % [m]
        alpha_NN= 0.05; % prob of discard good features in NN
        T_newLM= 15; % Threshold in NIS to create a new landmark
        sig_minLM= 0.1; % minimum SD for the landmarks
        multFactorAccIMU= 30; % multiplicative factor for the accel SD
        multFactorGyroIMU= 30; % multiplicative factor for the gyros SD
        multFactorPoseGPS= 3; % multiplicative factor for the GPS pose SD
        multFactorVelGPS= 20;  % multiplicative factor for the GPS velocity SD
        % -------------------------------------------
    end
    
    properties
        % --------------- Build parameters ---------------
        numEpochInclCalibration
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
    end
    methods
        function obj = ParametersClass()
            % --------------- Build parameters ---------------
            obj.numEpochInclCalibration= round(obj.numEpochStatic);
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
            %            obj.R_yaw_fn= @(v) sig_yaw_fn(v)^2;
            obj.T_NN= 4.5; %chi2inv(1-obj.alpha_NN,2);
            xPlot= [-0.3; 0; -0.3];
            yPlot= [0.1; 0; -0.1];
            zPlot= [0; 0; 0];
            obj.xyz_B= [xPlot, yPlot, zPlot]';
            obj.R_minLM= obj.sig_minLM.^2;
            % ----------------------------------------------
        end
    end
    
end