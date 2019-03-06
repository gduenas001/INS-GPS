classdef FGDataInputClass < handle
    properties
        gps_msmt
        gps_R
        lidar
        imu
        pose
        associations
    end
    
    methods
        function obj= FGDataInputClass(num_readings)
            % allocate memory
            obj.lidar= cell(1, num_readings);
            obj.imu= cell(1, num_readings);
            obj.gps_msmt= cell(1, num_readings);
            obj.gps_R= cell(1, num_readings);
            obj.pose= cell(1, num_readings);
            obj.associations= cell(1, num_readings);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function delete_fields_corresponding_to_static_epochs(obj, lidar)
            obj.lidar(1:lidar.index_of_last_static_lidar_epoch)= [];
            obj.imu(1:lidar.index_of_last_static_lidar_epoch)= [];
            obj.gps_msmt(1:lidar.index_of_last_static_lidar_epoch)= [];
            obj.gps_R(1:lidar.index_of_last_static_lidar_epoch)= [];
            obj.pose(1:lidar.index_of_last_static_lidar_epoch)= [];
            obj.associations(1:lidar.index_of_last_static_lidar_epoch)= [];
        end
    end
end