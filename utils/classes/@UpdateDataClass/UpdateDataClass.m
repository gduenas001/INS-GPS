
classdef UpdateDataClass < handle
    properties
        XX
        PX
        time
        number_of_associated_LMs
        
    end
    
    methods
        function obj= UpdateDataClass(num_readings)
            % allocate memory
            obj.XX= zeros(15, num_readings);
            obj.PX= zeros(15, num_readings);
            obj.time= zeros(num_readings, 1);
            obj.number_of_associated_LMs=zeros(num_readings, 1);
        end
        
        function store(obj, epoch, estimator, time)
            obj.XX(:,epoch)= estimator.XX(1:15);
            obj.PX(:,epoch)= diag( estimator.PX(1:15,1:15) ); % store only variances
            obj.time(epoch)= time;
            obj.number_of_associated_LMs(epoch)= estimator.number_of_associated_LMs;
        end
        
        function delete_extra_allocated_memory(obj, counters)
            obj.XX(:, counters.k_update+1:end)= [];
            obj.PX(:, counters.k_update+1:end)= [];
            obj.time( counters.k_update+1:end)= [];
            obj.number_of_associated_LMs( counters.k_update+1:end)= [];
            
        end
    end
end


