
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')


% create objects
params= ParametersClass("simulation");
im= IntegrityMonitoringClass(params);
estimator= EstimatorClass([], params);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
for epoch= 1:params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- IMU -------------
    estimator.odometry_update_sim( params );
    % -------------------------------
    
    % Store data
    data_obj.store_prediction_sim(epoch, estimator, counters.time_sim);
    
    % ------------------- GPS -------------------
    if params.SWITCH_GPS_UPDATE
        % GPS update 
        estimator.gps_update_sim( params );
    end
    % ----------------------------------------
    
    % ------------- LIDAR -------------
    if params.SWITCH_LIDAR_UPDATE
        
        % Simulate lidar feature measurements
        z_lidar= estimator.get_lidar_msmt_sim( params );
        
        if ~isempty(lidar.msmt)
            
            % NN data association
            association= estimator.nearest_neighbor_localization(z_lidar, params);
            
            % Evaluate the probability of mis-associations
            im.prob_of_MA_temporary( estimator, association, params);
            
            % Lidar update
            estimator.lidar_update_localization(lidar.msmt(:,1:2), association, params);
            
            % Lineariza and discretize
            estimator.linearize_discretize( imu.msmt(:,epoch+1), params.dt_imu, params); %Osama
            
            % integrity monitoring
            im.monitor_integrity(estimator, counters, data_obj, params);
            
            % Store data
            data_obj.store_msmts( body2nav(lidar.msmt, estimator.XX(1:9)) );% Add current msmts in Nav-frame
            counters.k_update= data_obj.store_update(counters.k_update, estimator, counters.time_sim);
            
            % increase integrity counter
            counters.increase_integrity_monitoring_counter();
        end
    end
    % ----------------------------------------
    
    % store udpate
    counters.k_update= data_obj.store_update(epoch, estimator, counters.time_sim);
end
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------




% Store data for last epoch
data_obj.store_update(counters.k_update, estimator, counters.time_sim);
data_obj.delete_extra_allocated_memory(counters)


% ------------- PLOTS -------------
% data_obj.plot_map(gps, imu.num_readings, params)
data_obj.plot_map_localization(estimator, gps, imu.num_readings, params)
data_obj.plot_number_of_landmarks_in_preceding_horizon();
data_obj.plot_estimates();
data_obj.plot_integrity_risk();
% ------------------------------------------------------------


