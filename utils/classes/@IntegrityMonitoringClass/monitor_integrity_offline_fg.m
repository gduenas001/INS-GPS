
function monitor_integrity_offline_fg(obj, estimator, counters, data,  params)


if params.SWITCH_FIXED_LM_SIZE_PH
    
    % current horizon measurements
    obj.n_M= sum( obj.n_ph ) + estimator.n_k;
    obj.n_L_M= obj.n_M / params.m_F;
    
    obj.M = obj.M + 1;
    
end

% monitor integrity if the number of LMs in the preceding horizon is more than threshold
if  ( params.SWITCH_FIXED_LM_SIZE_PH &&...
    obj.n_L_M >= params.min_n_L_M && obj.M > 1 ) ||...
    ( ~params.SWITCH_FIXED_LM_SIZE_PH &&...
    counters.k_im > obj.M )

    % Modify preceding horizon to have enough landmarks
    if params.SWITCH_FIXED_LM_SIZE_PH
        
        obj.n_M= estimator.n_k;
        for i= 1:length(obj.n_ph)
            obj.n_M= obj.n_M + obj.n_ph(i);
            % if the preceding horizon is long enough --> stop
            if obj.n_M >= params.min_n_L_M * params.m_F, break, end
        end
        % set the variables
        obj.n_L_M= obj.n_M / params.m_F;
        obj.M= i + 1;
        
    else
        
        % number of absolute msmts over the horizon
        obj.n_M= estimator.n_k + sum( obj.n_ph(1:obj.M - 1) );

        % number of landmarks over the horizon
        obj.n_L_M= obj.n_M / params.m_F;        
        
    end

    % compute extraction vector
    alpha= [ zeros( obj.M * params.m, 1 );...
            -sin( estimator.x_true(params.ind_yaw) );...
             cos( estimator.x_true(params.ind_yaw) );...
             0 ];
       
    % total number of msmts (prior + relative + abs)
    obj.n_total= obj.n_M + (obj.M + 1) * params.m;
    
    % number of states to estimate
    obj.m_M= (obj.M + 1) * params.m;
    
    % compute the H whiten Jacobian A
    obj.compute_whiten_jacobian_A(estimator, params);
    
    % construct the information matrix
    obj.Gamma_fg= obj.A' * obj.A;
    
    % full covarince matrix
    obj.PX_M= inv(obj.Gamma_fg);
    
    % extract covarince matrix at time k
    estimator.PX= obj.PX_M( end - params.m + 1 : end, end - params.m + 1 : end );
    
    % find the prior covarince matrix for time k+1
    obj.PX_prior= obj.PX_M( params.m + 1 : 2*params.m, params.m + 1 : 2*params.m );
    
    % initialization of p_hmi
    obj.p_hmi=0;
    if obj.n_M < 5    
        % if we don't have enough landmarks --> P(HMI)= 1
        obj.p_hmi= 1;
        
    else % we have enough msmts
        
        % Least squares residual matrix
        obj.M_M= eye( obj.n_total ) - obj.A * obj.PX_M * obj.A';
        
        % standard deviation in the state of interest
        obj.sigma_hat= sqrt( (alpha' / obj.Gamma_fg) * alpha );
        
        % number of hypotheses (just one per abs msmt for now)
        obj.n_H= obj.n_L_M;
        
        % set detector threshold from the continuity req
        obj.T_d= chi2inv( 1 - obj.C_req, obj.n_M );
        
        % fault probability of each association in the preceding horizon
        obj.P_F_M= ones(obj.n_L_M, 1) * params.P_UA;
        
        % initializing P_H vector
        obj.P_H= ones(obj.n_H, 1) * inf;
        
        for i= 0:0%obj.n_H
            
            % build extraction matrix
            if i == 0
                obj.compute_E_matrix_fg( 0, params.m_F);
            else
                obj.compute_E_matrix_fg( i, params.m_F);
            end
            
            % compute P(HMI | H) for the worst-case fault
            p_hmi_H= obj.compute_p_hmi_H(alpha, params);
            
            % Add P(HMI | H) to the integrity risk
            if i == 0
                obj.P_H_0= prod( 1 - obj.P_F_M );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H_0;
            else
                obj.P_H(i)= prod( obj.P_F_M( i ) );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H(i);
            end
        end
    end
    
    % store integrity related data
    data.store_integrity_data(obj, estimator, counters, params) 
end

% update the preceding horizon
update_preceding_horizon(obj, estimator, params)

end