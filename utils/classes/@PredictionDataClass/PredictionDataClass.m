
classdef PredictionDataClass < handle
    properties
        XX
        time
    end
    
    methods
        function obj= PredictionDataClass(num_readings)
            % allocate memory
            
            obj.XX= zeros(15, num_readings);
            obj.time= zeros(num_readings, 1);
        end
        
        function store(obj, epoch, estimator, time)
            obj.XX(:,epoch)= estimator.XX(1:15);
            obj.time(epoch)= time;
        end
    end
end


