
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')

lm_dens= 0.002;

% create objects
params= ParametersClass("simulation_Particle_filter",lm_dens);
estimator= EstimatorClassParticleSim(params);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
epoch= 1;

while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
%for epoch= 1:params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update( params );
    % -------------------------------
    
    % Store data
    data_obj.store_prediction_sim(epoch, estimator, counters.time_sim);
    
    % ----------------- LIDAR ----------------
    if params.SWITCH_LIDAR_UPDATE
        
        % Simulate lidar feature measurements
        z_lidar= estimator.get_lidar_msmt( params );
        
        estimator.association= estimator.association_true;
        
        % % NN data association
        % estimator.nearest_neighbor(z_lidar, params);
        
        % Lidar update
        estimator.lidar_update(z_lidar, params);
        
        % Add current msmts in Nav-frame
        data_obj.store_msmts( body2nav_2D(z_lidar, estimator.XX, estimator.XX(3)) ); 
        
        % Store data
        counters.k_update=...
            data_obj.store_update_fg(counters.k_update, estimator, counters.time_sim, params);
        
    end
    % -----------------------------------------
    
    % increase time
    counters.increase_time_sum_sim(params);
    counters.increase_time_sim(params);
    epoch= epoch + 1;
    
    if (trace(estimator.PX) < eps)
        estimator.XX_particles = mvnrnd(estimator.XX,eye(estimator.m)*0.2,estimator.number_of_particles);
        for i=1:estimator.number_of_particles
        	estimator.XX_particles(i,params.ind_yaw)= pi_to_pi(estimator.XX_particles(i,params.ind_yaw));
        end
        estimator.XX = mean(estimator.XX_particles);
        estimator.PX = cov(estimator.XX_particles);
    end
end
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------




% Store data for last epoch
data_obj.delete_extra_allocated_memory(counters)


% ------------- PLOTS -------------
% data_obj.plot_map_localization_sim(estimator, params.num_epochs_sim, params)
data_obj.plot_map_localization_sim_fg(estimator, params)
data_obj.plot_number_of_landmarks(params);
% data_obj.plot_number_epochs_in_preceding_horizon(params);
% data_obj.plot_estimates();
%data_obj.plot_integrity_risk(params);
% data_obj.plot_MA_probabilities();
data_obj.plot_error(params);
% data_obj.plot_P_H();
%data_obj.plot_detector(params);
% ------------------------------------------------------------



