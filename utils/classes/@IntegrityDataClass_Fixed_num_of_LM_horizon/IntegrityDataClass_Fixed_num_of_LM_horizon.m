
classdef IntegrityDataClass_Fixed_num_of_LM_horizon < handle
    properties 
        detector
        detector_threshold
        p_hmi
        n_L_M_for_LM
        M_for_LM
        sigma_hat
        time
        
        p_eps % prob that the estimate is out of bounds w/out faults
    end
    
    methods
        function obj= IntegrityDataClass_Fixed_num_of_LM_horizon(n)
            % allocate memory for approximately the number of readings
            
            obj.detector= zeros(n,1);
            obj.detector_threshold= zeros(n,1);
            obj.p_hmi= zeros(n,1);
            obj.n_L_M_for_LM= zeros(n,1);
            obj.sigma_hat= zeros(n,1);
            obj.time= zeros(n,1);
            obj.p_eps= zeros(n,1);
            obj.M_for_LM= zeros(n,1);
        end
        
        function store(obj, im, counters, params)
            obj.detector(counters.k_im)= im.q_M;
            obj.detector_threshold(counters.k_im)= im.detector_threshold;
            obj.p_hmi(counters.k_im)= im.p_hmi;
            obj.n_L_M_for_LM(counters.k_im)= im.n_L_M_for_LM;
            obj.sigma_hat(counters.k_im)= im.sigma_hat;
            obj.time(counters.k_im)= counters.time_sim;
            
            obj.p_eps(counters.k_im)= 2* normcdf(-params.alert_limit, 0, im.sigma_hat);
            obj.M_for_LM(counters.k_im)= im.M_for_LM;
        end
        
        function delete_extra_allocated_memory(obj, counters)
            obj.detector(counters.k_im:end)= [];
            obj.detector_threshold(counters.k_im:end)= [];
            obj.p_hmi(counters.k_im:end)= [];
            obj.n_L_M_for_LM(counters.k_im:end)= [];
            obj.sigma_hat(counters.k_im:end)= [];
            obj.time(counters.k_im:end)= [];
            obj.p_eps(counters.k_im:end)= [];
            obj.M_for_LM(counters.k_im:end)= [];
        end
    end
    
    
end





