
classdef IntegrityMonitoringClass < handle
    properties (Constant)
        m= 3
        calculate_A_M_recursively = 0;
    end
    properties (SetAccess = immutable)
        C_req
    end
    properties
        
        p_hmi
        detector_threshold
        
        is_extra_epoch_needed= -1 % initialize as (-1), then a boolean
        ind_im= [1,2,9];

        n_ph
        n_F_ph % number of features associated in the preceding horizon
        
        % (maybe unnecessary)
        E
        B_bar
        
        % current-time (k) only used when needed to extract elements
        sigma_hat
        Phi_k
        H_k
        L_k
        Lpp_k
        
        % augmented (M) 
        M= 0  % size of the preceding horizon in epochs
        n_M   % num msmts in the preceding horizon (including k)
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
        P_MA_M
        P_MA_k
        P_MA_ph
        T_d
        
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= IntegrityMonitoringClass(params)
            
            % if the preceding horizon is fixed in epochs --> set M
            if ~params.SWITCH_FIXED_LM_SIZE_PH
                obj.M= params.preceding_horizon_size;
            end
            
            % if it's a simulation --> change the indexes
            if params.SWITCH_SIM, obj.ind_im= 1:3; end
            
            % continuity requirement
            obj.C_req= params.continuity_requirement;
            
            % initialize the preceding horizon 
            obj.n_ph=     zeros(params.preceding_horizon_size,1);
            obj.Phi_ph=   cell(1, params.preceding_horizon_size + 1); % need an extra epoch here
            obj.gamma_ph= cell(1, params.preceding_horizon_size);
            obj.q_ph=     ones(params.preceding_horizon_size, 1) * (-1);
            obj.L_ph=     cell(1, params.preceding_horizon_size);
            obj.Lpp_ph=   cell(1, params.preceding_horizon_size + 1); % need an extra epoch here (osama)
            obj.H_ph=     cell(1, params.preceding_horizon_size);
            obj.Y_ph=     cell(1, params.preceding_horizon_size);
            obj.P_MA_ph=  cell(1, params.preceding_horizon_size);
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function neg_p_hmi= optimization_fn(obj, f_M_mag, fx_hat_dir, M_dir, sigma_hat, l, dof)
            neg_p_hmi= - ((1 - normcdf(l , f_M_mag * fx_hat_dir, sigma_hat) +...
                normcdf(-l , f_M_mag * fx_hat_dir, sigma_hat))...
                * ncx2cdf(obj.detector_threshold, dof, f_M_mag^2 * M_dir ));
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_E_matrix(obj, i, m_F)
            if i == 0 % E matrix for only previous state faults
                obj.E= zeros( obj.m, obj.n_M + obj.m );
                obj.E(:, end-obj.m + 1:end)= eye(obj.m);
            else % E matrix with faults in the PH
                obj.E= zeros( obj.m + m_F , obj.n_M + obj.m );
                obj.E( end-obj.m+1 : end , end-obj.m+1:end )= eye(obj.m); % previous bias
                obj.E( 1:m_F , (i-1)*m_F + 1 : i*m_F )= eye(m_F); % landmark i faulted
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        monitor_integrity(obj, estimator, counters, data, params)
        % ----------------------------------------------
        % ----------------------------------------------
%         prob_of_MA(obj, estimator, params);
        % ----------------------------------------------
        % ----------------------------------------------
        prob_of_MA_temporary(obj, estimator, association, params);
        % ----------------------------------------------
        % ----------------------------------------------
        compute_Y_M_matrix(obj,estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_A_M_matrix(obj,estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_B_bar_matrix(obj, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        function update_preceding_horizon(obj, estimator, params)
            
            if params.SWITCH_FIXED_LM_SIZE_PH
                obj.n_ph=     [estimator.n_k;     obj.n_ph(1:obj.M)];
                obj.gamma_ph= {estimator.gamma_k, obj.gamma_ph{1:obj.M}};
                obj.q_ph=     [estimator.q_k;     obj.q_ph(1:obj.M)];
                obj.Phi_ph=   {obj.Phi_k,         obj.Phi_ph{1:obj.M+ 1}}; %%%%%%%% CAREFUL
                obj.H_ph=     {obj.H_k,           obj.H_ph{1:obj.M}};
                obj.L_ph=     {obj.L_k,           obj.L_ph{1:obj.M}};
                obj.Lpp_ph=   {obj.Lpp_k,         obj.Lpp_ph{1:obj.M}};
                obj.Y_ph=     {estimator.Y_k,     obj.Y_ph{1:obj.M}};
                obj.P_MA_ph=  {obj.P_MA_k,        obj.P_MA_ph{1:obj.M}};
                
            else
                obj.n_ph=     [estimator.n_k;     obj.n_ph(1:end-1)];
                obj.gamma_ph= {estimator.gamma_k, obj.gamma_ph{1:end-1}};
                obj.q_ph=     [estimator.q_k;     obj.q_ph(1:end-1)];
                obj.Phi_ph=   {obj.Phi_k,         obj.Phi_ph{1:end-1}}; %%%%%%%% CAREFUL
                obj.H_ph=     {obj.H_k,           obj.H_ph{1:end-1}};
                obj.L_ph=     {obj.L_k,           obj.L_ph{1:end-1}};
                obj.Lpp_ph=   {obj.Lpp_k,         obj.Lpp_ph{1:end-1}};
                obj.Y_ph=     {estimator.Y_k,     obj.Y_ph{1:end-1}};
                obj.P_MA_ph=  {obj.P_MA_k,        obj.P_MA_ph{1:end-1}};
            end
        end
        
    end
end




