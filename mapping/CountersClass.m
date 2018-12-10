
classdef CountersClass < handle
    properties
        time_sim
        time_sum= 0
        time_sum_virt_z= 0
        time_sum_virt_y= 0
        time_gps
        timeLIDAR
        
        k_update= 1
        k_gps= 1
        k_lidar= 1
    end
    
    methods
        function obj= CountersClass(gps, lidar)
            
            obj.time_gps= gps.time(1); % this is zero, as the GPS time is the reference
            obj.timeLIDAR= lidar.time(1,2);
            
        end
        
        function increase_gps_counter(obj)
            obj.k_gps= obj.k_gps + 1;
        end
        
        function increase_lidar_counter(obj)
            obj.k_lidar= obj.k_lidar + 1;
        end
    end
end






