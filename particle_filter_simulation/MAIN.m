
clear; format short; clc; % close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')

figure; hold on; grid on;

lm_dens= 0.002;

% create objects
params= ParametersClass("simulation_Particle_filter",lm_dens);
estimator= EstimatorClassParticleSim(params);
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
%for epoch= 1:params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update( params );
    % -------------------------------
    
    if epoch > 1
        delete(XX_particles);
    end
    
    plot(estimator.x_true(1), estimator.x_true(2), 'r.','markersize', 7);
    plot(estimator.XX(1), estimator.XX(2), 'g.','markersize', 7);
    XX_particles= plot( estimator.XX_particles(:,1) , estimator.XX_particles(:,2), 'b.','markersize',3 );
    pause(.1)
    
    % Store data
    data_obj.store_prediction_sim(epoch, estimator, counters.time_sim);
    
    if (trace(estimator.PX) < 0.5)
        XX_particles_new = mvnrnd(estimator.XX,estimator.gen_estimate_cov*5,estimator.number_of_particles_to_add);
        for i=1:estimator.number_of_particles_to_add
        	XX_particles_new(i,params.ind_yaw)= pi_to_pi(estimator.XX_particles(i,params.ind_yaw));
        end
        estimator.XX_particles=[estimator.XX_particles;XX_particles_new];
        estimator.XX = mean(estimator.XX_particles)';
        estimator.PX = cov(estimator.XX_particles);
    end
    
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
    
    XX_particles= plot( estimator.XX_particles(:,1) , estimator.XX_particles(:,2), 'b.','markersize',3 );
    plot(estimator.XX(1), estimator.XX(2), 'k.','markersize', 7);
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



