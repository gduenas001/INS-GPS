
clear all; close all; clc;

% integrity requirement
density= 1;
I_REQ= 1e-5;
num_maps= 10;
num_runs= 30;

% common paths for map and runs
path= strcat('../data/simulation/factor_graph/results/density_00', num2str(density), '_fix/map_');

availability= ones(num_maps,1) * inf;
hmi_map= ones(num_maps,1) * inf;
for map_i= 4:4%um_maps
    fprintf('map --> %d\n', map_i);
    
    % load the map
    file_name_map= strcat( path, num2str(map_i), '/offline.mat' );
    offline_data= load(file_name_map);
    
    % check availability
    num_epochs= length(offline_data.data_obj.im.p_hmi);
    avail_inds= offline_data.data_obj.im.p_hmi < I_REQ;
    availability(map_i)= sum( avail_inds ) / num_epochs;
    
    % create intervals of non-availability
    non_avail_inds= ~avail_inds;
    non_avail_times= offline_data.data_obj.im.time(non_avail_inds);
    
    % plot integrity risk & map
    offline_data.data_obj.plot_map_localization_sim_fg(offline_data.estimator, offline_data.params)
    offline_data.data_obj.plot_integrity_risk_fg(offline_data.params);

    % loop through runs
    hmi= ones(num_runs, 1);
    detection_inds= cell(1, num_runs);
    failure_inds= cell(1, num_runs);
    for run_i= 11:20%num_runs
        % load the run
        file_name_run= strcat( path, num2str(map_i), '/online_', num2str(run_i), '.mat' );
        load(file_name_run);
        
        % detection & estimate failures
        detection_inds{run_i}= data_obj.update.q_d > data_obj.update.T_d;
        failure_inds{run_i}= data_obj.update.error_state_interest > params.alert_limit;
        
%         if sum(failure_inds{run_i}) > 0
%             disp('a')
%         end

        % check HMI
        num_epochs= length(data_obj.update.q_d);
        hmi_ind= ~detection_inds{run_i} & failure_inds{run_i};
        hmi_times= data_obj.update.time(hmi_ind);
        hmi(run_i)= sum(hmi_ind) / num_epochs;
        
        % check the P(HMI) at those times
        if hmi(run_i) > 0
            data_obj.plot_number_of_landmarks_fg_sim(params);
            data_obj.plot_detector_fg(params);
            data_obj.plot_error_fg(params);


            close; close; close;
            if ~ismember( hmi_times, non_avail_times)
                disp('not working well')
            end
        end
    end
    
    close; close;
    
    % average HMI
    hmi_map(map_i)= mean(hmi);
end

% average availability
ave_availability= mean(availability)
ave_hmi= mean(hmi_map)

%%

data_obj= offline_data.data_obj;
estimator= offline_data.estimator;
params= offline_data.params;

