
classdef UpdateDataClass < handle
    properties
        XX
        PX
        time
        
    end
    
    methods
        function obj= UpdateDataClass(num_readings)
            % allocate memory
            
            obj.XX= zeros(15, num_readings);
            obj.PX= zeros(15, num_readings);
            obj.time= zeros(num_readings, 1);
        end
        
        function store(obj, epoch, estimator, time)
            obj.XX(:,epoch)= estimator.XX(1:15);
            obj.PX(:,epoch)= diag( estimator.PX(1:15,1:15) ); % store only variances
            obj.time(epoch)= time;
        end
    end
end


