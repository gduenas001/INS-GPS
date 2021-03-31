
classdef IntegrityMonitoringClassEkfSLAMSim < handle
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
        ind_im= 1:3;

        
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
        n_L_M% num landmarks in the preceding horizon (including k)
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
        Gamma_prior
        lidar_msmt_ind
        gps_msmt_ind
        n_gps_ph % number of gps msmt at each epoch in PH
        H_gps_ph
        H_lidar_ph
        n_M_gps
        A_reduced
        min_f_dir_vs_M_dir
        f_mag
        
        noncentral_dof= cell(10000,1)
        f_dir_sig2= cell(10000,1)
        M_dir= cell(10000,1)
        counter_H=0
        p_hmi_elapsed_time=0
        f_avg=0
        D_k
        Delta_k
        num_of_states_to_track
        n_k
        n_L_k
        S_k
        M_k
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= IntegrityMonitoringClassEkfSLAMSim(params, estimator)
            
            % if the preceding horizon is fixed in epochs --> set M
            if params.SWITCH_FIXED_LM_SIZE_PH
                obj.M= 0;
            else
                obj.M= params.M;
            end
            
            % continuity requirement
            obj.C_req= params.continuity_requirement;
            
            % initialize the preceding horizon
            obj.n_ph=     zeros( params.M, 1 );
            obj.Phi_ph=   cell( 1, params.M + 1 ); % need an extra epoch here
            obj.H_ph=     cell( 1, params.M );
            obj.gamma_ph= cell(1, params.M);
            obj.q_ph=     ones(params.M, 1) * (-1);
            obj.L_ph=     cell(1, params.M);
            obj.Lpp_ph=   cell(1, params.M + 1); % need an extra epoch here (osama)
            obj.Y_ph=     cell(1, params.M);
            obj.P_MA_ph=  cell(1, params.M);
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function neg_p_hmi= optimization_fn(obj, f_M_mag, fx_hat_dir, M_dir, sigma_hat, l, dof)
            neg_p_hmi= - ( (normcdf(l , f_M_mag * fx_hat_dir, sigma_hat, 'upper') +...
                normcdf(-l , f_M_mag * fx_hat_dir, sigma_hat))...
                * ncx2cdf(obj.T_d, dof, f_M_mag.^2 * M_dir ) );
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_E_matrix(obj, i, m_F)
            if sum(i) == 0 % E matrix for only previous state faults
                obj.E= zeros( obj.m, obj.n_k + obj.num_of_states_to_track );
                %obj.E(:, end-obj.num_of_states_to_track + 1:end)= eye(obj.num_of_states_to_track);
                obj.E(:, obj.n_k + (1:obj.m))= eye(obj.m);
            else % E matrix with faults in the PH
                obj.E= zeros( obj.m + m_F*length(i) , obj.n_k + obj.num_of_states_to_track );
                obj.E( end-obj.m+1:end , obj.n_k + (1:obj.m) )= eye(obj.m); % previous bias
                for j= 1:length(i)
                    obj.E( m_F*(j-1)+1 : m_F*j , (i(j)-1)*m_F + 1 : i(j)*m_F )= eye(m_F); % landmark i(j) faulted
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
        compute_Y_M_matrix(obj,estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_A_M_matrix(obj,estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_B_bar_matrix(obj, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        alpha= build_state_of_interest_extraction_matrix(obj, params, current_state)
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




