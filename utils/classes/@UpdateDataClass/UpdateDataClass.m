
classdef UpdateDataClass < handle
    properties
        x_true
        error
        XX
        PX
        time
        miss_associations
        num_associated_lms
        num_of_extracted_features
        q_d % detectors
        T_d % detector thresholds
        n_L_k % number of associated features at k
        n_L_M % number of associated features in the ph
    end
    
    methods
        function obj= UpdateDataClass(num_readings, params)
            % allocate memory
            obj.x_true= zeros(params.m, num_readings);
            obj.error= zeros(params.m, num_readings);
            obj.XX= zeros(params.m, num_readings);
            obj.PX= zeros(params.m, num_readings);
            obj.time= zeros(num_readings, 1);
            obj.miss_associations= zeros(num_readings, 1);
            obj.num_associated_lms= zeros(num_readings, 1);
            obj.num_of_extracted_features= zeros(num_readings, 1);
            obj.q_d= zeros(num_readings, 1);
            obj.T_d= zeros(num_readings, 1); 
            obj.n_L_M= zeros(num_readings, 1);
            obj.n_L_k= zeros(num_readings, 1);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store(obj, epoch, estimator, time)
            obj.XX(:,epoch)= estimator.XX(1:15);
            obj.PX(:,epoch)= diag( estimator.PX(1:15,1:15) ); % store only variances
            obj.time(epoch)= time;
            obj.num_associated_lms(epoch)= estimator.num_associated_lms;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_sim(obj, epoch, estimator, time, params)
            obj.error(:,epoch)= estimator.XX - estimator.x_true;
            obj.XX(:,epoch)= estimator.XX;
            obj.PX(:,epoch)= diag( estimator.PX ); % store only variances
            obj.time(epoch)= time;
            obj.miss_associations(epoch)= sum( boolean(...
                (estimator.association ~= estimator.association_true) .* estimator.association) );
            obj.num_associated_lms(epoch)= estimator.num_associated_lms;
            obj.num_of_extracted_features(epoch)= estimator.num_of_extracted_features;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_fg(obj, epoch, estimator, time, params)
            obj.x_true(:,epoch)= estimator.x_true;
            obj.XX(:,epoch)= estimator.XX;
            obj.time(epoch)= time;
            obj.num_associated_lms(epoch)= estimator.n_L_k;
            obj.q_d(epoch)= estimator.q_d;
            obj.T_d(epoch)= estimator.T_d;
            obj.n_L_k(epoch)= estimator.n_L_k;
            obj.n_L_M(epoch)= estimator.n_L_M;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function delete_extra_allocated_memory(obj, counters)
            obj.XX(:, counters.k_update+1:end)= [];
            obj.PX(:, counters.k_update+1:end)= [];
            obj.time( counters.k_update+1:end)= [];
            obj.num_associated_lms( counters.k_update+1:end)= [];
            obj.q_d(counters.k_update+1:end)= [];
            obj.T_d(counters.k_update+1:end)= [];
            obj.n_L_k(counters.k_update+1:end)= [];
            obj.n_L_M(counters.k_update+1:end)= [];
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end


