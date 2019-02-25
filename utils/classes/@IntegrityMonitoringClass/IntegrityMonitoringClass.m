
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

        
        % (maybe unnecessary)
        E
        B_bar
        
        % hypotheses
        inds_H % faulted indexes under H hypotheses
        P_H_0
        P_H
        T_d
        n_H
        n_max
        
        % for MA purposes
        mu_k
        kappa

        % current-time (k) only used when needed to extract elements
        sigma_hat
        Phi_k
        H_k
        L_k
        Lpp_k
        P_MA_k
        P_MA_k_full
        
        % augmented (M) 
        M= 0  % size of the preceding horizon in epochs
        n_M   % num msmts in the preceding horizon (including k) -if FG ---> num abs msmts
        n_L_M % num landmarks in the preceding horizon (including k)
        Phi_M
        q_M
        gamma_M
        Y_M
        A_M
        M_M
        P_MA_M
        P_F_M
        
        % preceding horizon saved (ph)
        Phi_ph
        q_ph
        gamma_ph
        A_ph
        L_ph
        Lpp_ph
        H_ph
        Y_ph
        P_MA_ph
        n_ph
        n_F_ph % number of features associated in the preceding horizon

        
        % Factor Graph variables
        m_M       % number of states to estimate
        n_total   % total number of msmts (prior + relative + abs)
        XX_ph
        D_bar_ph
        A
        Gamma_fg % information matrix
        M_fg
        PX_prior
        PX_M
        abs_msmt_ind
        faulted_LMs_indices
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= IntegrityMonitoringClass(params, estimator)
            
            % if the preceding horizon is fixed in epochs --> set M
            if params.SWITCH_FIXED_LM_SIZE_PH
                obj.M= 0;
            else
                obj.M= params.preceding_horizon_size;
            end
            
            % if it's a simulation --> change the indexes
            if params.SWITCH_SIM, obj.ind_im= 1:3; end
            
            % continuity requirement
            obj.C_req= params.continuity_requirement;
            
            % initialize the preceding horizon
            % TODO: should this change for a fixed horizon in landmarks?
            if params.SWITCH_SIM && params.SWITCH_FACTOR_GRAPHS
                obj.XX_ph=     cell(1, params.preceding_horizon_size+1);
                obj.XX_ph{1}=  estimator.XX;
                obj.D_bar_ph=  cell(1, params.preceding_horizon_size);
                obj.PX_prior=  estimator.PX;
            end
            obj.n_ph=     zeros( params.preceding_horizon_size, 1 );
            obj.Phi_ph=   cell( 1, params.preceding_horizon_size + 1 ); % need an extra epoch here
            obj.H_ph=     cell( 1, params.preceding_horizon_size );
            obj.gamma_ph= cell(1, params.preceding_horizon_size);
            obj.q_ph=     ones(params.preceding_horizon_size, 1) * (-1);
            obj.L_ph=     cell(1, params.preceding_horizon_size);
            obj.Lpp_ph=   cell(1, params.preceding_horizon_size + 1); % need an extra epoch here (osama)
            obj.Y_ph=     cell(1, params.preceding_horizon_size);
            obj.P_MA_ph=  cell(1, params.preceding_horizon_size);
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function neg_p_hmi= optimization_fn(obj, f_M_mag, fx_hat_dir, M_dir, sigma_hat, l, dof)
            neg_p_hmi= - ( (1 - normcdf(l , f_M_mag * fx_hat_dir, sigma_hat) +...
                normcdf(-l , f_M_mag * fx_hat_dir, sigma_hat))...
                * ncx2cdf(obj.T_d, dof, f_M_mag.^2 * M_dir ) );
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_E_matrix(obj, i, m_F)
            if sum(i) == 0 % E matrix for only previous state faults
                obj.E= zeros( obj.m, obj.n_M + obj.m );
                obj.E(:, end-obj.m + 1:end)= eye(obj.m);
            else % E matrix with faults in the PH
                obj.E= zeros( obj.m + m_F*length(i) , obj.n_M + obj.m );
                obj.E( end-obj.m+1 : end , end-obj.m+1:end )= eye(obj.m); % previous bias
                for j= 1:length(i)
                    obj.E( m_F*(j-1)+1 : m_F*j , (i(j)-1)*m_F + 1 : i(j)*m_F )= eye(m_F); % landmark i(j) faulted
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_E_matrix_fg(obj, i, m_F)
            if sum(i) == 0 % E matrix for only previous state faults
                obj.E= zeros( obj.m, obj.n_total );
                obj.E(:, 1:obj.m)= eye(obj.m);
            else % E matrix with a single LM fault
                obj.E= zeros( obj.m + m_F*length(i) , obj.n_total );
                obj.E( 1:obj.m , 1:obj.m )= eye(obj.m); % previous bias
                for j= 1:length(i)
                    ind= obj.abs_msmt_ind(:,i(j));
                    obj.E( obj.m + 1 + m_F*(j-1) : obj.m + m_F*(j) , ind(:)' )= eye(m_F); % landmark i faulted
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        monitor_integrity(obj, estimator, counters, data, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_hypotheses(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        prob_of_MA(obj, estimator, association, params);
        % ----------------------------------------------
        % ----------------------------------------------
        compute_hypotheses_probabilities(obj, params);
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
        monitor_integrity_offline_fg(obj, estimator, counters, data,  params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_whiten_jacobian_A(obj, estimator, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_required_epochs_for_min_LMs(obj, params, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        alpha= build_state_of_interest_extraction_matrix(obj, params, current_state)
        % ----------------------------------------------
        % ----------------------------------------------
        p_hmi_H= compute_p_hmi_H(obj, alpha, params)
        % ----------------------------------------------
        % ----------------------------------------------
        function update_preceding_horizon(obj, estimator, params)
            
            % TODO: organize 
            if params.SWITCH_FACTOR_GRAPHS
                obj.Phi_ph=   {inf, estimator.Phi_k, obj.Phi_ph{2:obj.M}};
                obj.H_ph=     {estimator.H_k,   obj.H_ph{1:obj.M-1}};
                obj.n_ph=     [estimator.n_k;   obj.n_ph(1:obj.M-1)];
                obj.XX_ph=    {estimator.XX,    obj.XX_ph{1:obj.M}};
                obj.D_bar_ph= {inf, estimator.D_bar, obj.D_bar_ph{2:obj.M}};
            else
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
end




