
classdef IntegrityMonitoringClass < handle
    properties (Constant)
        m= 3
        p_H= 1e-3
        ind_im= [1,2,9];
    end
    properties (SetAccess = immutable)
        M % size of the preceding horizon in epochs
        C_req
    end
    properties
        
        p_hmi
        detector_threshold
        
        n_ph
        n_F_ph % number of features associated in the preceding horizon
        
        % (maybe unnecessary)
        B_bar
        
        % current-time (k) only used when needed to extract elements        
        PX
        Phi_k
        H_k
        L_k
        Lpp_k
        q_k
        
        % augmented (M) 
        n_M % num msmts in the preceding horizon (including k)
        n_L_M % num landmarks in the preceding horizon (including k)
        Phi_M
        q_M
        gamma_M
        Y_M
        A_M
        M_M
        
        % preceding horizon saved (ph)
        Phi_ph
        q_ph
        gamma_ph
        A_ph
        L_ph
        Lpp_ph
        H_ph
        Y_ph
        
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= IntegrityMonitoringClass(params)
            obj.M= params.preceding_horizon_size;
            
            obj.n_ph=     ones(obj.M,1) * (-1);
            obj.Phi_ph=   cell(1, obj.M + 1); % need an extra epoch here
            obj.q_ph=   cell(1, obj.M);
            obj.gamma_ph= cell(1, obj.M);
            obj.L_ph=     cell(1, obj.M);
            obj.Lpp_ph=   cell(1, obj.M);
            obj.H_ph=     cell(1, obj.M);
            obj.Y_ph=     cell(1, obj.M);
            
            obj.C_req= params.continuity_requirement;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        compute_A_M_matrix(obj)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_B_bar_matrix(obj, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        function update_preceding_horizon(obj, estimator)
            
            obj.n_ph=     [estimator.n_k; obj.n_ph(1:end-1)];
            obj.gamma_ph= [estimator.gamma_k, obj.gamma_ph(1:end-1)];
            obj.Phi_ph=   [obj.Phi_k^12, obj.Phi_ph(1:end-1)]; %%%%%%%% CAREFUL
            obj.H_ph=     [obj.H_k, obj.H_ph(1:end-1)];
            obj.L_ph=     [obj.L_k, obj.L_ph(1:end-1)];
            obj.Lpp_ph=   [obj.Lpp_k, obj.Lpp_ph(1:end-1)];
            obj.Y_ph=     [estimator.Y_k, obj.Y_ph(1:end-1)];
        end
        
    end
end




