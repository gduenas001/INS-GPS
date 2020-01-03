
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')

%lm_dens=[0.001,0.005,0.009,0.013,0.017,0.021,0.025,0.029,0.033,0.037];
%lm_dens=0.003:0.006:0.025;
%lm_dens=linspace(0.002,0.025,10);
%lm_dens=[0.002,0.011,0.02];
lm_dens=[0.002,0.004,0.008];
computational_time = zeros(1,length(lm_dens));
avg_n_L_M = zeros(1,length(lm_dens));
avg_f_mag = zeros(1,length(lm_dens));
avg_epoch = zeros(1,length(lm_dens));
%for map_i= 10:10
for ind_lm_dens= 1:1%length(lm_dens)
    
% seed the randomness
map_i= 8
rng(map_i)
    
% create objects
params= ParametersClass("simulation_fg_offline_MA",lm_dens(ind_lm_dens));
estimator= EstimatorClassFgSimOffMA(params);
im= IntegrityMonitoringClassFgSimOffMA(params, estimator);
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
    estimator.odometry_update(params);
    % -------------------------------
    
    % ----------------- LIDAR ----------------
     if params.SWITCH_LIDAR_UPDATE

         % build the jacobian landmarks in the field of view
         estimator.compute_lidar_H_k( params );
         
         % main function for factor graphs integrity monitoring
         im.monitor_integrity(estimator, counters, data_obj,  params);
         estimator.M = im.M;
         % Store data
         counters.k_update=...
             data_obj.store_update_fg(counters.k_update, estimator, counters.time_sim, params);
         
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
ind_lm_dens
computational_time(ind_lm_dens)=mean(data_obj.im.p_hmi_elapsed_time(data_obj.im.p_hmi_elapsed_time~=0));
avg_n_L_M(ind_lm_dens)=mean(data_obj.im.n_L_M(data_obj.im.n_L_M~=0));
avg_f_mag(ind_lm_dens)=mean(data_obj.im.f_avg(data_obj.im.f_avg~=0));
avg_epoch(ind_lm_dens)=mean(data_obj.update.M(data_obj.update.M~=0)+1);

save(['PLANS_P_MA_example_',num2str(lm_dens(ind_lm_dens)),'_two_simult_faults.mat']);
% save workspace
%save(strcat( params.path_sim_fg, 'results/density_001/map_', num2str(map_i), '/offline' ));

end


% -------------------------- PLOTS --------------------------
data_obj.plot_map_localization_sim_fg(estimator, params)
data_obj.plot_number_of_landmarks_fg_sim(params);
data_obj.plot_integrity_risk(params);
data_obj.plot_alert_limit_over_sig_hat(params)
% ------------------------------------------------------------
