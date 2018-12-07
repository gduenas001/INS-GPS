
clear; format short; clc; close all;
dbstop if error

addpath('../utils/')

% create objects
params= ParametersClass();
gps= GPSClass(params.numEpochStatic * params.dT_IMU, params);
lidar= LidarClass(params, gps.timeInit);
imu= IMUClass(params, gps.timeInit);
estimator= EstimatorClass(imu.msmt(1:3, params.numEpochInclCalibration), params);
data_obj= DataClass(imu.num_readings);


% Initialize loop variables
timeSum= 0;
timeSumVirt_Z= 0;
timeSumVirt_Y= 0;
timeGPS= gps.time(1); % this is zero, as the GPS time is the reference
timeLIDAR= lidar.time(1,2);
k_update= 1;
k_GPS= 1;
k_LIDAR= 1;

GPS_Index_exceeded = 0;   % TODO: osama is this needeed?
LIDAR_Index_exceeded = 0; % TODO: osama is this needeed?

% Initial discretization for cov. propagation
estimator.linearize_discretize( imu.msmt(:,1), params.dT_IMU, params );

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
for epoch= 1:imu.num_readings-1
    disp(strcat('Epoch -> ', num2str(epoch)));
    
    % set the simulation time to the IMU time
    timeSim= imu.time(epoch);
    
    % Turn off GPS updates if start moving
    if epoch == params.numEpochStatic
        params.SWITCH_CALIBRATION= 0; 
        estimator.PX(7,7)= params.sig_phi0^2;
        estimator.PX(8,8)= params.sig_phi0^2;
        estimator.PX(9,9)= params.sig_yaw0^2;
    end
        
    % Increase time count
    timeSum= timeSum + params.dT_IMU;
    timeSumVirt_Z= timeSumVirt_Z + params.dT_IMU;
    timeSumVirt_Y= timeSumVirt_Y + params.dT_IMU;
    
    % ------------- IMU -------------
    if ~params.SWITCH_CALIBRATION
        estimator.imu_update( imu.msmt(:,epoch), ...
            params.taua_normal_operation, params.tauw_normal_operation, params );
    end
    % -------------------------------
    
    % Store data
    data_obj.store_prediction(epoch, estimator, timeSim);
    
    % ------------- Calibration -------------
    if timeSum >= params.dT_cal && params.SWITCH_CALIBRATION
        
        % create a fake msmt and do a KF update to set biases 
        estimator.calibration(imu.msmt(:,epoch), params); 
        
        % Store data
        k_update= data_obj.store_update(k_update, estimator, timeSim);

        % Reset counter
        timeSum= 0;
    end
    % ------------------------------------
    
    % ------------- virtual msmt update >> Z vel  -------------  
    if timeSumVirt_Z >= params.dT_virt_Z && params.SWITCH_VIRT_UPDATE_Z && ~params.SWITCH_CALIBRATION
        
        zVelocityUpdate( params.R_virt_Z);
        
        % Reset counter
        timeSumVirt_Z= 0;
    end
    % ---------------------------------------------------------
    
    % ------------- virtual msmt update >> Y vel  -------------  
    if timeSumVirt_Y >= params.dT_virt_Y && params.SWITCH_VIRT_UPDATE_Y && ~params.SWITCH_CALIBRATION
         
        % Yaw update
        if params.SWITCH_YAW_UPDATE && norm(estimator.XX(4:6)) > params.minVelocityYaw
            disp('yaw udpate');
            estimator.yaw_update( imu.msmt(4:6,epoch), params);
        end
        
        % Reset counter
        timeSumVirt_Y= 0;
    end
    % ---------------------------------------------------------
    
    
    % ------------------- GPS -------------------
    if (timeSim + params.dT_IMU) > timeGPS && ~GPS_Index_exceeded
        
        if ~params.SWITCH_CALIBRATION && params.SWITCH_GPS_UPDATE
            % GPS update -- only use GPS vel if it's fast
            estimator.gps_update( gps.msmt(:,k_GPS), gps.R(:,k_GPS), params);
            
            % Yaw update
            if params.SWITCH_YAW_UPDATE && norm(estimator.XX(4:6)) > params.minVelocityYaw
                disp('yaw udpate');
                estimator.yaw_update( imu.msmt(4:6,epoch), params );
            end
            estimator.linearize_discretize( imu.msmt(:,epoch), params.dT_IMU, params);

            % Store data
            k_update= data_obj.store_update(k_update, estimator, timeSim);
        end
        
        % Time GPS counter
        k_GPS= k_GPS + 1;
        
        % -----Osama----- TODO: Osama what is this??
        if k_GPS <= size(gps.time,1)
            timeGPS= gps.time(k_GPS);
        else
           k_GPS = k_GPS -1 ;
           GPS_Index_exceeded = 1;
        end
    end
    % ----------------------------------------
    
    % ------------- LIDAR -------------
    if (timeSim + params.dT_IMU) > timeLIDAR && params.SWITCH_LIDAR_UPDATE
        
        if epoch > params.numEpochStatic + 1500 % TODO: osama, why are you adding 1500
            % Read the lidar features
            epochLIDAR= lidar.time(k_LIDAR,1);
            lidar.get_msmt( epochLIDAR, params );
            
            % Remove people-features for the data set 
            for i= 1:size(lidar.areas_to_remove,1)
                lidar.remove_features_in_area(estimator.XX(1:9), lidar.areas_to_remove(i,:));
            end
                        
            % NN data association
            [association]= estimator.nearest_neighbor(lidar.msmt(:,1:2), params);
            
            % Lidar update
            estimator.lidarUpdate(lidar.msmt(:,1:2), association, ...
                params.R_lidar, params.SWITCH_CALIBRATION);

            % Increase landmark covariance to the minimum
            estimator.increase_landmarks_cov(params.R_minLM);
            
            % Add new landmarks
            estimator.addNewLM( lidar.msmt(association' == -1,:), params.R_lidar );
            
            % Lineariza and discretize
            estimator.linearize_discretize( imu.msmt(:,epoch), params.dT_IMU, params);
            
            % Store data
            % Add the current msmts in Nav-frame to plot
            data_obj.store_msmts( body2nav(lidar.msmt, estimator.XX(1:9)) );
            k_update= data_obj.store_update(k_update, estimator, timeSim);
        end
        
        % Increase counters
        k_LIDAR= k_LIDAR + 1;
        
        % -----Osama----- TODO: osama, what is this again?
        if k_LIDAR <= length(lidar.time)
            timeLIDAR= lidar.time(k_LIDAR,2);
        else
           k_LIDAR = k_LIDAR -1 ;
           LIDAR_Index_exceeded = 1;
        end
    end
    % ---------------------------------
    
end
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------








% Store data for last epoch
data_obj.store_update(k_update, estimator, timeSim);
data_obj.remove_extra_allocated_memory(k_update)


% ------------- PLOTS -------------
data_obj.plot_map(gps, imu.num_readings, params)
data_obj.plot_estimates();
% ------------------------------------------------------------








