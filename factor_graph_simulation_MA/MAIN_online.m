clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')
pause on

%lm_dens=[0.001,0.005,0.009,0.013,0.017,0.021,0.025,0.029,0.033,0.037];
%lm_dens=0.005:0.005:0.025;
lm_dens=[0.001,0.0025,0.0065];
computational_time = zeros(1,length(lm_dens));
avg_n_L_M = zeros(1,length(lm_dens));
avg_epoch = zeros(1,length(lm_dens));
avg_q_d = zeros(1,length(lm_dens));
for ind_lm_dens= 1:length(lm_dens)

% seed the randomness
map_i= 6;
rng(map_i)
    
% create objects
params= ParametersClass("simulation_fg_online_MA",lm_dens(ind_lm_dens));
estimator= EstimatorClassFgSimOnMA([], params);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);



% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
epoch= 1; % initialize time index

while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update( params );
    % ------------------------------------
    
    % ------------- Gyro -------------
    if epoch > 1
        estimator.generate_gyro_msmt(...
            data_obj.update.x_true(params.ind_yaw,epoch-1), estimator.x_true(params.ind_yaw), params);
    end
    % --------------------------------
    
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % get the lidar msmts
         estimator.get_lidar_msmt(params);
         estimator.association= estimator.association_true;
                  
         % solve the fg optimization
         estimator.solve(counters, params)
         
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

computational_time(ind_lm_dens)=mean(data_obj.update.detector_elapsed_time(data_obj.update.detector_elapsed_time~=0));
avg_n_L_M(ind_lm_dens)=mean(data_obj.update.n_L_M(data_obj.update.n_L_M~=0));
avg_epoch(ind_lm_dens)=mean(data_obj.update.M(data_obj.update.M~=0)+1);
avg_q_d(ind_lm_dens)=mean(data_obj.update.q_d(data_obj.update.q_d~=0));
% Store data for last epoch
data_obj.delete_extra_allocated_memory(counters)

save(['PLANS_online_P_MA_example_',num2str(lm_dens(ind_lm_dens)),'_two_simult_faults_n_max_2_corrected_scratch_new.mat']);

end

% -------------------------- PLOTS --------------------------
data_obj.plot_map_localization_sim_fg(estimator, params)
data_obj.plot_number_of_landmarks_fg_sim(params);
data_obj.plot_detector(params);
data_obj.plot_error(params);
% ------------------------------------------------------------

data_obj.find_HMI_sim(params)

