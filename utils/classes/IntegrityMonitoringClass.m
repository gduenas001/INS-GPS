
classdef IntegrityMonitoringClass < handle
    
    properties
        A_M
        
        q_D_k
        q_D_M
        gamma_k
        gamma_M
        
        L_k
        L_M
        Lpp_k
        Lpp_M
        H_k
        H_M
        Y_k
        Y_M
        
    end
    
    properties (SetAccess = immutable)
        M % size of the preceding horizon in epochs
    end
    
    methods
        function obj= IntegrityMonitoringClass(params)
            obj.M= params.preceding_horizon_size;
            
            obj.q_D_M= cell(obj.M,1);
            obj.gamma_M= cell(obj.M,1);
            obj.L_M= cell(obj.M,1);
            obj.Lpp_M= cell(obj.M,1);
            obj.H_M= cell(obj.M,1);
            
            
        end
        
        function store_data_initial(obj)
        end
    end
end




