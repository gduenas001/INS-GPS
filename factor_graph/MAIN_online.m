clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')


% create objects
params= ParametersClass("simulation_fg_online");
estimator= EstimatorClass([], params);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);

% initialize time index
epoch= 1;

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update_sim( params );
    % ------------------------------------
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % get the lidar msmts
         estimator.get_lidar_msmt_sim(params);
         
         % build the jacobian landmarks in the field of view
         estimator.update_fg( counters, params );
         
         % Store data
         counters.k_update=...
             data_obj.store_update_fg(counters.k_update, estimator, counters.time_sim, params);
         
         % increase counter
         counters.increase_lidar_counter();
         
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


% -------------------------- PLOTS --------------------------
data_obj.plot_map_localization_sim(estimator, params.num_epochs_sim, params)
% data_obj.plot_number_of_landmarks(params);
% ------------------------------------------------------------



