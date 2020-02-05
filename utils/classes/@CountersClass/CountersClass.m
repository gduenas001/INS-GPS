
classdef CountersClass < handle
    properties
        time_sim= 0
        time_sum= 0
        time_sum_virt_z= 0
        time_sum_virt_y= 0
        time_gps
        time_lidar
        
        k_update= 1
        k_gps= 1
        k_lidar= 1
        k_im= 1;
    end
    
    methods
        function obj= CountersClass(gps, lidar, params)
            
            % if it's a simulation --> time is not read from files
            if ~params.SWITCH_SIM && ~params.SWITCH_FACTOR_GRAPHS && ~params.SWITCH_SM
                obj.time_gps= gps.time(1); % this is zero, as the GPS time is the reference
                obj.time_lidar= lidar.time(1,2);
            elseif (~params.SWITCH_SIM && ~params.SWITCH_FACTOR_GRAPHS && params.SWITCH_SM)
                obj.time_lidar= lidar.time(1,2);
            end
        end
        
        function increase_gps_counter(obj)
            obj.k_gps= obj.k_gps + 1;
        end
        
        function increase_lidar_counter(obj)
            obj.k_lidar= obj.k_lidar + 1;
        end
        
        function increase_integrity_monitoring_counter(obj)
            obj.k_im= obj.k_im + 1;
        end
        
        function increase_time_sums(obj, params)
            obj.time_sum= obj.time_sum + params.dt_imu;
            obj.time_sum_virt_z= obj.time_sum_virt_z + params.dt_imu;
            obj.time_sum_virt_y= obj.time_sum_virt_y + params.dt_imu;           
        end
        
        function increase_time_sum_sim(obj, params)
            obj.time_sum= obj.time_sum + params.dt_sim;
        end
        
        function increase_time_sim(obj, params)
            obj.time_sim= obj.time_sim + params.dt_sim;
        end
        
        function reset_time_sum(obj)
            obj.time_sum= 0;
        end
        
        function reset_time_sum_virt_z(obj)
            obj.time_sum_virt_z= 0;
        end
        
        function reset_time_sum_virt_y(obj)
            obj.time_sum_virt_y= 0;
        end
    end
end






