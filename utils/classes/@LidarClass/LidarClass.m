
classdef LidarClass < handle
    
    properties (SetAccess = immutable)
        time
        num_readings
        R
        areas_to_remove
    end
    
    properties (SetAccess = private) % this can only be modified with its own methods
        msmt   % this only stores the current msmt -->  private
    end
    
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= LidarClass(params, init_time)
            % substracts the GPS start time which is used as reference
            
            % load the variable "T_LIDAR"
            obj.time= load(strcat(params.file_name_lidar_path,'T_LIDAR.mat'));
            obj.time= obj.time.T_LIDAR;
            
            % load the areas from where we remove people
            try
                obj.areas_to_remove= load(strcat(params.file_name_lidar_path,'areas_to_remove.mat'));
                obj.areas_to_remove= obj.areas_to_remove.areas_to_remove;
            end
            
            % Use the GPS first reading time as reference
            obj.time(:,2)= obj.time(:,2) - init_time;
            
            % If some of the initial times are negative (prior to first GPS reading) --> eliminate them
            obj.time(obj.time(:,2) < 0 , :)= []; 
            
            % number of lidar scans
            obj.num_readings= length(obj.time);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function get_msmt(obj, epoch, params)
            % load the mat file with the extrated features at the lidar epoch specified
            
            fileName= strcat(params.file_name_lidar_path,'matFiles/Epoch',num2str(epoch),'.mat');
            
            % loads the z variable with range and bearing
            obj.msmt= load(fileName);
            obj.msmt= obj.msmt.z;
            
            % if there are features --> prepare the measurement
            if ~isempty(obj.msmt)
                if params.SWITCH_REMOVE_FAR_FEATURES
                    obj.remove_far_features(params.lidarRange);
                end
                
                % Add height
                obj.msmt= [obj.msmt, ones(size(obj.msmt,1),1) * params.feature_height];
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function remove_far_features(obj, lidarRange)
            % removes extracted features farther than "lidarRange"
            
            d= obj.msmt(:,1).^2 + obj.msmt(:,2).^2;
            obj.msmt( d > lidarRange^2, :)= [];
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function remove_features_in_areas(obj, x)
            % remove features from areas, each area is: [minX, maxX, minY, maxY]
            
            % Remove people-features for the data set 
            for i= 1:size(obj.areas_to_remove,1)
                
                area= obj.areas_to_remove(i,:);
                
                % transform to nav-frame first
                msmt_nav_frame= body2nav(obj.msmt,x);
                
                % Remove people-features
                inX= (msmt_nav_frame(:,1) > area(1)) & (msmt_nav_frame(:,1) < area(2));
                inY= (msmt_nav_frame(:,2) > area(3)) & (msmt_nav_frame(:,2) < area(4));
                obj.msmt( inX & inY, :)= [];
            end        
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end















