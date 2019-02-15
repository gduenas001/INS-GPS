
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')


% create objects
params= ParametersClass("Factor_Graph");
estimator= EstimatorClass([], params);
im= IntegrityMonitoringClass(params,estimator);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);

% initialize time index
epoch= 1;

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
while ~estimator.goal_is_reached
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_exact_prediction_FG( params );
    % -------------------------------
    
    % Store data
    data_obj.store_prediction_FG(epoch, estimator, counters.time_sim);
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % Evaluate the jacobian matrix of FoV LMs
         estimator.get_lidar_jacobian_for_FoV_LMs_FG( params );

         im.FG_covarience_update_and_integrity_monitoring(estimator, counters, data_obj,  params);

         % Store data
         counters.k_update=...
             data_obj.store_update_FG(counters.k_update, estimator, counters.time_sim, params);
         
         % increase integrity counter
         counters.increase_integrity_monitoring_counter();
     end
    % -----------------------------------------
    
    % increase time
    counters.increase_time_sum_sim(params);
    counters.increase_time_sim(params);
    epoch= epoch + 1;
end
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------




% Store data for last epoch
data_obj.delete_extra_allocated_memory(counters)


% ------------- PLOTS -------------
data_obj.plot_map_localization_FG(estimator, params.num_epochs_sim, params)
data_obj.plot_number_of_landmarks(params);
data_obj.plot_integrity_risk(params);
% ------------------------------------------------------------



