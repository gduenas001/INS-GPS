
classdef IntegrityMonitoringClass < handle
    
    properties
        p_hmi
        
        Phi_M
        
        q_D_k
        q_D_M
        gamma_k
        gamma_M
        
        A_M
        
        L_M
        
        Lpp_M
        
        H_M
        
        Y_M
        
    end
    
    properties (SetAccess = immutable)
        M % size of the preceding horizon in epochs
    end
    
    methods
        function obj= IntegrityMonitoringClass(params)
            obj.M= params.preceding_horizon_size;
            
            obj.Phi_M= cell(obj.M,1);
            obj.q_D_M= cell(obj.M,1);
            obj.gamma_M= cell(obj.M,1);
            obj.L_M= cell(obj.M,1);
            obj.Lpp_M= cell(obj.M,1);
            obj.H_M= cell(obj.M,1);
            obj.Y_M= cell(obj.M,1);
            
            
        end
        
        function monitor_integrity(obj, estimator, counters)
            
            % only store data if there are not enough epochs
            if counters.k_im <= obj.M
                
                obj.Phi_M= [estimator.Phi_k, obj.Phi_M(1:end-1)];
                obj.H_M= [estimator.H_k, obj.H_M(1:end-1)];
                obj.L_M= [estimator.L_k, obj.L_M(1:end-1)];
                obj.Y_M= [estimator.Y_k, obj.Y_M(1:end-1)];
                
                
            end
        end
    end
end




