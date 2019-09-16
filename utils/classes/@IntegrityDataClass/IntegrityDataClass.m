
classdef IntegrityDataClass < handle
    properties 
        
        time % time at which each data has been stored
        association % associations without zeros
        association_full % associations at k w/o LM selection
        P_MA_k % probability of missassociations at k
        P_MA_k_full % probability of missassociations at k w/o LM selection
        P_H % hypotheses probabilities at k
        detector
        detector_threshold
        p_hmi
        p_hmi_elapsed_time
        n_L_M
        M
        sigma_hat
        f_avg
        p_eps % prob that the estimate is out of bounds w/out faults
    end
    
    methods
        function obj= IntegrityDataClass(n)
            % allocate memory for approximately the number of readings
            obj.association= cell(n, 1); 
            obj.association_full= cell(n, 1); 
            obj.P_MA_k= cell(n, 1);
            obj.P_MA_k_full= cell(n, 1); 
            obj.P_H= cell(n, 1); 
            obj.detector= zeros(n,1);
            obj.detector_threshold= zeros(n,1);
            obj.p_hmi= zeros(n,1);
            obj.p_hmi_elapsed_time= zeros(n,1);
            obj.n_L_M= zeros(n,1);
            obj.sigma_hat= zeros(n,1);
            obj.time= zeros(n,1);
            obj.p_eps= zeros(n,1);
            obj.M= zeros(n,1);
            obj.f_avg= zeros(n,1);
        end
        
        function store(obj, im, estimator, counters, params)
            obj.p_hmi(counters.k_im)= im.p_hmi;
            obj.p_hmi_elapsed_time(counters.k_im)=im.p_hmi_elapsed_time;
            obj.n_L_M(counters.k_im)= im.n_L_M;
            obj.P_H{counters.k_im}= im.P_H;
            obj.detector_threshold(counters.k_im)= im.T_d;
            obj.sigma_hat(counters.k_im)= im.sigma_hat;
            obj.time(counters.k_im)= counters.time_sim;
            obj.p_eps(counters.k_im)= 2* normcdf(-params.alert_limit, 0, im.sigma_hat);
            obj.M(counters.k_im)= im.M;
            obj.f_avg(counters.k_im)= im.f_avg;
            
            
            if ~params.SWITCH_FACTOR_GRAPHS      
                obj.association{counters.k_im}= estimator.association_no_zeros;
                obj.association_full{counters.k_im}= estimator.association_full;
                obj.P_MA_k{counters.k_im}= im.P_MA_k;
                obj.P_MA_k_full{counters.k_im}= im.P_MA_k_full;    
                obj.detector(counters.k_im)= im.q_M;
            end
        end
        
        function delete_extra_allocated_memory(obj, counters)
            obj.association(counters.k_im:end)= [];
            obj.association_full(counters.k_im:end)= [];
            obj.P_MA_k(counters.k_im:end)= [];
            obj.P_MA_k_full(counters.k_im:end)= [];
            obj.P_H(counters.k_im:end)= [];
            obj.detector(counters.k_im:end)= [];
            obj.detector_threshold(counters.k_im:end)= [];
            obj.p_hmi(counters.k_im:end)= [];
            obj.p_hmi_elapsed_time(counters.k_im:end)= [];
            obj.n_L_M(counters.k_im:end)= [];
            obj.sigma_hat(counters.k_im:end)= [];
            obj.time(counters.k_im:end)= [];
            obj.p_eps(counters.k_im:end)= [];
            obj.M(counters.k_im:end)= [];
            obj.f_avg(counters.k_im:end)= [];
        end
    end
    
    
end





