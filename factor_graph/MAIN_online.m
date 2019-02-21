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
    
    % ------------- Gyro -------------
    if epoch > 1
        estimator.generate_gyro_msmt_sim(...
            data_obj.update.x_true(params.ind_yaw,epoch-1), estimator.x_true(params.ind_yaw), params);
    else
        estimator.z_gyro= 0;
    end
    % --------------------------------
    
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % get the lidar msmts
         estimator.get_lidar_msmt_sim(params);
         estimator.association= estimator.association_true;
         
         % update fg msmt vector with all types of msmts
         estimator.update_z_fg(counters, params);
         
         % solve the fg optimization
         estimator.solve_fg(counters, params)
         
         
         
                  
         

         
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % update odometry msmts in the ph
         estimator.odometry_ph= {inf, estimator.odometry_k, estimator.odometry_ph{2:params.M}};
         
         % update odometry msmts in the ph
         estimator.z_gyro_ph= {inf, estimator.z_gyro, estimator.z_gyro_ph{2:params.M}};
         
         % update lidar msmts in the ph
         z_lidar= estimator.z_lidar';
         estimator.z_lidar_ph= {z_lidar(:), estimator.z_lidar_ph{1:params.M}};
         
         % update the previous poses
         estimator.x_ph= {estimator.XX, estimator.x_ph{1:params.M}};
         
         % update the associations in the ph
         estimator.association_ph= {estimator.association, estimator.association_ph{1:params.M}};
         
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
         
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



