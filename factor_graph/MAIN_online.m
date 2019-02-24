clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')


% create objects
params= ParametersClass("simulation_fg_online");
estimator= EstimatorClass([], params);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);


% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
epoch= 1; % initialize time index

while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update_sim_fg( params );
    % ------------------------------------
    
    % ------------- Gyro -------------
    if epoch > 1
        estimator.generate_gyro_msmt_sim(...
            data_obj.update.x_true(params.ind_yaw,epoch-1), estimator.x_true(params.ind_yaw), params);
    end
    % --------------------------------
    
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % get the lidar msmts
         estimator.get_lidar_msmt_sim(params);
         estimator.association= estimator.association_true;
                  
         % solve the fg optimization
         estimator.solve_fg(counters, params)
         
         % update preceding horizon for the estimate
         estimator.update_preceding_horizon(params)
         
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
data_obj.plot_map_localization_sim_fg(estimator, params)
data_obj.plot_number_of_landmarks_fg(params);
% data_obj.plot_detector_fg(params);
data_obj.plot_error_fg(params);
% ------------------------------------------------------------



