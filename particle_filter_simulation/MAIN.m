
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')

figure; hold on; grid on;

lm_dens= 0.002; % 0.002;

seed= 7;
rng(seed);

% create objects
params= ParametersClass("simulation_Particle_filter",lm_dens);
estimator= EstimatorClassParticleSim(params);
im= IntegrityMonitoringClassPfSim(params, estimator);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);

set(gca,'FontSize',15)
xlabel('X [m]','FontSize',15); ylabel('Y [m]','FontSize',15); zlabel('Z [m]','FontSize',15);

plot(params.way_points(1,:), params.way_points(2,:), 'bo', 'markersize', 5);
pause(.1)

% create a map of landmarks
lm_map= [estimator.landmark_map(:,1),...
    estimator.landmark_map(:,2),...
    zeros(estimator.num_landmarks,1)];
plot(lm_map(:,1), lm_map(:,2), 'r+', 'markersize', 2.5);
pause(.1)

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
epoch= 1;

while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
    
    % Generate more particles if the percentage of distinct particles is very low
    if (length(unique(estimator.particles_indices_update))/length(estimator.particles_indices_update) < estimator.threshold_add_particles)%0.01%0.5
       XX_particles_new = mvnrnd(estimator.XX_update,estimator.gen_estimate_cov,estimator.number_of_particles_to_add);%XX_particles_new = mvnrnd(estimator.XX,estimator.gen_estimate_cov*5,estimator.number_of_particles_to_add);
       for i=1:estimator.number_of_particles_to_add
       	XX_particles_new(i,params.ind_yaw)= pi_to_pi(estimator.XX_particles_update(i,params.ind_yaw));
       end
       %estimator.XX_particles_update=[estimator.XX_particles_update;XX_particles_new];
       estimator.XX_particles_update=XX_particles_new;
       %estimator.particles_indices_update = [estimator.particles_indices_update; transpose(max(estimator.particles_indices_update)+1:max(estimator.particles_indices_update)+estimator.number_of_particles_to_add) ];
       estimator.particles_indices_update = transpose( 1:size(obj.XX_particles_predict,1) );
       estimator.XX_update = mean(estimator.XX_particles_update)';
       %[~, indices_of_unique_particles, ~] = unique(estimator.particles_indices_update);
       %estimator.SX_update = cov(estimator.XX_particles_update(indices_of_unique_particles,:));
       estimator.SX_update = cov(estimator.XX_particles_update);
    end
    estimator.XX_particles_prior = estimator.XX_particles_update;
    estimator.particles_indices_prior = estimator.particles_indices_update;
    estimator.XX_prior = estimator.XX_update;
    estimator.SX_prior = estimator.SX_update;
    estimator.XX_particles_predict = estimator.XX_particles_update;
    estimator.particles_indices_predict = estimator.particles_indices_update;
    estimator.XX_predict = estimator.XX_update;
    estimator.SX_predict = estimator.SX_update;
    %estimator.prior_estimate_cov = estimator.SX_update;
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update( params );
    % -------------------------------
    
    if epoch > 1
        delete(XX_particles);
    end
    
    plot(estimator.x_true(1), estimator.x_true(2), 'r.','markersize', 7);
    plot(estimator.XX_predict(1), estimator.XX_predict(2), 'g.','markersize', 7);
    XX_particles= plot( estimator.XX_particles_predict(:,1) , estimator.XX_particles_predict(:,2), 'b.','markersize',3 );
    pause(.1)
    
    % Store data
    data_obj.store_prediction_sim_pf(epoch, estimator, counters.time_sim);
    
    estimator.XX_particles_update = estimator.XX_particles_predict;
    estimator.particles_indices_update = estimator.particles_indices_predict;
    estimator.XX_update = estimator.XX_predict;
    estimator.SX_update = estimator.SX_predict;
    
    % ----------------- LIDAR ----------------
    if params.SWITCH_LIDAR_UPDATE
        
        % Simulate lidar feature measurements
        z_lidar= estimator.get_lidar_msmt( params );
        
        estimator.association= estimator.association_true;
        
        % % NN data association
        % estimator.nearest_neighbor(z_lidar, params);
        
        % Lidar update
        estimator.lidar_update(z_lidar, params);
        
        im.monitor_integrity(estimator, counters, data_obj,  params);
        
        % Add current msmts in Nav-frame
        data_obj.store_msmts( body2nav_2D(z_lidar, estimator.XX_update, estimator.XX_update(3)) ); 
        
        % Store data
        counters.k_update=...
            data_obj.store_update_sim_pf(counters.k_update, estimator, counters.time_sim, params);
        
        % increase integrity counter
        counters.increase_integrity_monitoring_counter();
        
        if epoch > 1
            delete(msmts);
        end

        msmts = plot(data_obj.msmts(:,1), data_obj.msmts(:,2), 'k.');
        pause(.1)
        
    end
    % -----------------------------------------
    
    % increase time
    counters.increase_time_sum_sim(params);
    counters.increase_time_sim(params);
    epoch= epoch + 1;
    
    
    if epoch > 1
        delete(XX_particles);
    end
    
    XX_particles= plot( estimator.XX_particles_update(:,1) , estimator.XX_particles_update(:,2), 'b.','markersize',3 );
    plot(estimator.XX_update(1), estimator.XX_update(2), 'k.','markersize', 7);
    pause(.1)
    
    
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


