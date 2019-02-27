
clear;% close all; clc;

% integrity requirement
I_REQ= 1e-5;
num_maps= 10;
num_runs= 30;

% common paths for map and runs
path= '../data/simulation/factor_graph/results/density_004_fix/map_';

availability= ones(num_maps,1) * inf;
hmi_map= ones(num_maps,1) * inf;
for map_i= 1:num_maps
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
    
    
    
%     hmi= ones(num_runs, 1);
%     detection_inds= cell(1, num_runs);
%     failure_inds= cell(1, num_runs);
%     for run_i= 1:num_runs
%         % load the run
%         file_name_run= strcat( path, num2str(map_i), '/online_', num2str(run_i), '.mat' );
%         load(file_name_run);
%         
%         % detection & estimate failures
%         detection_inds{run_i}= data_obj.update.q_d > data_obj.update.T_d;
%         failure_inds{run_i}= data_obj.update.error_state_interest > params.alert_limit;
%         
%         % check HMI
%         num_epochs= length(data_obj.update.q_d);
%         hmi_ind= detection_inds{run_i} & failure_inds{run_i};
%         hmi_times= data_obj.update.time(hmi_ind);
%         hmi(run_i)= sum(hmi_ind) / num_epochs;
%         
%         % check the P(HMI) at those times
%         if hmi(run_i) > 0
%             if ~ismember( hmi_times, non_avail_times)
%                 disp('not working well')
%             end
%         end
%         
% %     end
%     
%     % average HMI
%     hmi_map(map_i)= mean(hmi);
end

% average availability
ave_availability= mean(availability)
ave_hmi= mean(hmi_map)