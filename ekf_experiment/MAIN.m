
clear; format short; clc; close all;
dbstop if error

addpath('../utils/functions')
addpath('../utils/classes')

% Uncomment for revised IJRR maniscript
%set_of_min_n_L_M=[5 10 15 20 25 30];
%computational_time = zeros(1,length(set_of_min_n_L_M));
%avg_n_L_M = zeros(1,length(set_of_min_n_L_M));
%avg_f_mag = zeros(1,length(set_of_min_n_L_M));
%avg_epoch = zeros(1,length(set_of_min_n_L_M));
%for ind_min_n_L_M= 1:length(set_of_min_n_L_M)

% create objects
params= ParametersClass("localization_kf");
% Uncomment for revised IJRR maniscript
%params.min_n_L_M = set_of_min_n_L_M(ind_min_n_L_M);
gps= GPSClass(params.num_epochs_static * params.dt_imu, params);
lidar= LidarClass(params, gps.timeInit);
imu= IMUClass(params, gps.timeInit);
estimator= EstimatorClassEkfExp(imu.msmt(1:3, 1:params.num_epochs_static), params);
im= IntegrityMonitoringClassEkfExp(params, estimator);
data_obj= DataClass(imu.num_readings, lidar.num_readings, params);
counters= CountersClass(gps, lidar, params);
% Uncomment for FG
% FG= FGDataInputClass(lidar.num_readings);

% Initial discretization for cov. propagation
estimator.linearize_discretize( imu.msmt(:,1), params.dt_imu, params );

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
for epoch= 1:imu.num_readings - 1
    disp(strcat('Epoch -> ', num2str(epoch)));

    % set the simulation time to the IMU time
    counters.time_sim= imu.time(epoch);

    % Turn off GPS updates if start moving
    if epoch == params.num_epochs_static
        params.turn_off_calibration();
        estimator.PX(7,7)= params.sig_phi0^2;
        estimator.PX(8,8)= params.sig_phi0^2;
        estimator.PX(9,9)= params.sig_yaw0^2;
    end

    % Increase time count
    counters.increase_time_sums(params);

    % ------------- IMU -------------
    estimator.imu_update( imu.msmt(:,epoch), params );
    % -------------------------------

    % Store data
    data_obj.store_prediction(epoch, estimator, counters.time_sim);

    % ------------- Calibration -------------
    if counters.time_sum >= params.dt_cal && params.SWITCH_CALIBRATION

        % create a fake msmt and do a KF update to set biases 
        estimator.calibration(imu.msmt(:,epoch+1), params); %Osama 

        % Store data
        counters.k_update= data_obj.store_update(counters.k_update, estimator, counters.time_sim);
        counters.reset_time_sum();
    end
    % ------------------------------------

    % ------------- virtual msmt update >> Z vel  -------------  
    if counters.time_sum_virt_z >= params.dt_virt_z && params.SWITCH_VIRT_UPDATE_Z && ~params.SWITCH_CALIBRATION
        estimator.vel_update_z(params.R_virt_Z);
        counters.reset_time_sum_virt_z();
    end
    % ---------------------------------------------------------

    % ------------- virtual msmt update >> Y vel  -------------
    if counters.time_sum_virt_y >= params.dt_virt_y && params.SWITCH_VIRT_UPDATE_Y && ~params.SWITCH_CALIBRATION

        % Yaw update
        if params.SWITCH_YAW_UPDATE && norm(estimator.XX(4:6)) > params.min_vel_yaw
            disp('yaw update');
            estimator.yaw_update( imu.msmt(4:6,epoch+1), params); %Osama
        end
        counters.reset_time_sum_virt_y();
    end
    % ---------------------------------------------------------


    % ------------------- GPS -------------------
    if (counters.time_sim + params.dt_imu) > counters.time_gps 

        if ~params.SWITCH_CALIBRATION && params.SWITCH_GPS_UPDATE
            % GPS update -- only use GPS vel if it's fast
            estimator.gps_update( gps.msmt(:,counters.k_gps), gps.R(:,counters.k_gps), params);

            % This is used to store gps msmt and R recieved at lidar epoch for FG
            gps.IS_GPS_AVAILABLE= 1;
            current_gps_msmt= gps.msmt(:,counters.k_gps);
            current_gps_R= gps.R(:,counters.k_gps);

            % Yaw update
            if params.SWITCH_YAW_UPDATE && norm(estimator.XX(4:6)) > params.min_vel_yaw
                disp('yaw update');
                estimator.yaw_update( imu.msmt(4:6,epoch+1), params ); %Osama
            end
            estimator.linearize_discretize( imu.msmt(:,epoch+1), params.dt_imu, params); %Osama

            % Store data
            counters.k_update= data_obj.store_update( counters.k_update, estimator, counters.time_sim );
        end

        % Time GPS counter
        if counters.k_gps == gps.num_readings
            params.turn_off_gps();
        else
            counters.increase_gps_counter();
            counters.time_gps= gps.time(counters.k_gps);
        end
    end
    % ----------------------------------------

    % ------------- LIDAR -------------
    if (counters.time_sim + params.dt_imu) > counters.time_lidar && params.SWITCH_LIDAR_UPDATE

        if epoch > params.num_epochs_static
            % Read the lidar features
            epochLIDAR= lidar.time(counters.k_lidar,1);
            lidar.get_msmt( epochLIDAR, params );

            % Remove people-features for the data set
            lidar.remove_features_in_areas(estimator.XX(1:9));

            % NN data association
            estimator.nearest_neighbor(lidar.msmt(:,1:2), params);

            % Evaluate the probability of mis-associations
            im.prob_of_MA( estimator, params);

            % Lidar update
            estimator.lidar_update(lidar.msmt(:,1:2), params);

            % Lineariza and discretize
            estimator.linearize_discretize( imu.msmt(:,epoch+1), params.dt_imu, params); %Osama

            % Store the required data for Factor Graph
            z= lidar.msmt(:,1:2);
            z(estimator.association == 0, :)= [];

            % Uncomment for FG
            %FG.lidar{counters.k_lidar}= z;
            %FG.associations{counters.k_lidar}= estimator.association_no_zeros;
            %FG.imu{counters.k_lidar}= imu.msmt(:,epoch);
            %FG.pose{counters.k_lidar}= estimator.XX;
            if gps.IS_GPS_AVAILABLE
                % Uncomment for FG
                %FG.gps_msmt{counters.k_lidar}= current_gps_msmt;
                %FG.gps_R{counters.k_lidar}= current_gps_R;
                gps.IS_GPS_AVAILABLE= 0; % mark the current gps reading as stored
            end


            % integrity monitoring
            im.monitor_integrity(estimator, counters, data_obj, params);

            % Store data
            data_obj.store_msmts( body2nav_3D(lidar.msmt, estimator.XX(1:9)) );% Add current msmts in Nav-frame
            counters.k_update= data_obj.store_update(counters.k_update, estimator, counters.time_sim);

            % increase integrity counter
            counters.increase_integrity_monitoring_counter();

        else

            % Index of last static lidar epoch(it will be obatined during the run)
            lidar.index_of_last_static_lidar_epoch= counters.k_lidar;

        end

        % Time lidar counter
        if counters.k_lidar == lidar.num_readings
            params.turn_off_lidar; 
        else
            counters.increase_lidar_counter();
            counters.time_lidar= lidar.time(counters.k_lidar,2);
        end

    end
 end
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------

% Uncomment for IJRR revised manuscript
%computational_time(ind_min_n_L_M)=mean(data_obj.im.p_hmi_elapsed_time(data_obj.im.p_hmi_elapsed_time~=0));
%avg_n_L_M(ind_min_n_L_M)=mean(data_obj.im.n_L_M(data_obj.im.n_L_M~=0));
%avg_f_mag(ind_min_n_L_M)=mean(data_obj.im.f_avg(data_obj.im.f_avg~=0));
%avg_epoch(ind_min_n_L_M)=mean(data_obj.im.M(data_obj.im.M~=0)+1);


% Store data for last epoch
data_obj.store_update(counters.k_update, estimator, counters.time_sim);
data_obj.delete_extra_allocated_memory(counters)

% Uncomment for FG
% delete fields corresponding to static epochs
%FG.delete_fields_corresponding_to_static_epochs(lidar)

% Uncomment for IJRR revised manuscript
%end 


% ------------- PLOTS ------------
data_obj.plot_map_localization(estimator, gps, imu.num_readings, params)

data_obj.plot_number_of_landmarks(params);
data_obj.plot_number_epochs_in_preceding_horizon(params);
data_obj.plot_estimates();
data_obj.plot_integrity_risk(params);
data_obj.plot_MA_probabilities();
% data_obj.plot_P_H();
% --------------------------------


