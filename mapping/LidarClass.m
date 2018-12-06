
classdef LidarClass < handle
    
    properties (SetAccess = immutable)
        time
        R
    end
    
    properties (SetAccess = private)
        msmt   % this only stores the current msmt -->  private
    end
    
    
    
    methods
        function obj= LidarClass(params, init_time)
            % substracts the GPS start time which is used as reference
            
            % loads the variable "T_LIDAR"
            load(strcat(params.file_name_lidar_path,'T_LIDAR.mat'));
            obj.time= T_LIDAR;
            
            % Use the GPS first reading time as reference
            obj.time(:,2)= obj.time(:,2) - init_time;
            
            % If some of the initial times are negative (prior to first GPS reading) --> eliminate them
            obj.time(obj.time(:,2) < 0 , :)= [];
            
        end
        
        
        function get_msmt(obj, epoch, params)
            % load the mat file with the extrated features at the lidar epoch specified
            
            fileName= strcat(params.file_name_lidar_path,'matFiles/Epoch',num2str(epoch),'.mat');
            
            % loads the z variable with range and bearing
            obj.msmt= load(fileName);
            obj.msmt= obj.msmt.z;
            
            if params.SWITCH_REMOVE_FAR_FEATURES
                obj.removeFarFeatures(params.lidarRange);
            end
            
            % Add height
            height= 1.5;
            obj.msmt= [obj.msmt, ones(size(obj.msmt,1),1)*height];
            
        end
        
        
        function removeFarFeatures(obj, lidarRange)
            % removes extracted features farther than "lidarRange"
            
            d= obj.msmt(:,1).^2 + obj.msmt(:,2).^2;
            
            obj.msmt( d > lidarRange^2, :)= [];
        end
        
        
        function removeFeatureInArea(obj, x, minX, maxX, minY, maxY)
            
            zNav= obj.body2nav(obj.msmt,x);
            
            % Remove people-features
            inX= (zNav(:,1) > minX) & (zNav(:,1) < maxX);
            inY= (zNav(:,2) > minY) & (zNav(:,2) < maxY);
            
            obj.msmt( inX & inY, :)= [];
            
        end
        
        
        
        function z= body2nav(~,z,x)
            
            % Convert in 3D
            R_NB= R_NB_rot(x(7),x(8),x(9));
            z= ( R_NB * z' + x(1:3) )';
            
            % % Convert in 2D
            % z= z(:,1:2);
            % R_NB= R_NB_rot(0,0,x(9));
            % R_NB= R_NB(1:2,1:2);
            % z= ( R_NB * z' + x(1:2) )';
            
            % Put the back into 2D
            z= z(:,1:2);
            
        end
        
    end
end















