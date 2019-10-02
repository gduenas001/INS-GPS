
function monitor_integrity(obj, estimator, counters, data,  params)


% calculate the current number of LMs in PH; only before starting integrity monitoring
if params.SWITCH_FIXED_LM_SIZE_PH && isempty(obj.p_hmi)
    
    % current horizon measurements
    obj.n_M= sum( obj.n_ph(1:obj.M) ) + estimator.n_k;
    % current horizon LMs
    obj.n_L_M= obj.n_M / params.m_F;
    estimator.n_L_M= obj.n_L_M;
    % update the length of PH
    obj.M= obj.M +1; 
    
end

% monitor integrity if the number of abs msmts in PH is more than threshold,...
% Or if the number of LMs in PH is more than threshold, ...
% Or if the num of epochs in PH equals to a specific value
if  ( params.SWITCH_FIXED_LM_SIZE_PH ...
      && obj.n_L_M + ((estimator.n_gps_k + sum(obj.n_gps_ph))/6) >= params.min_n_L_M ...
      && (( (estimator.n_gps_k + sum(obj.n_gps_ph) ~= 0) ...
      && params.SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt)...
      || (~params.SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt)) ) ||...
    ( ~params.SWITCH_FIXED_LM_SIZE_PH && counters.k_im > obj.M )

    % Modify PH to have enough landmarks (in case of Fixed LM or abs msmts)
    if params.SWITCH_FIXED_LM_SIZE_PH
        
        obj.compute_required_epochs_for_min_LMs(params, estimator)
        
    else
        
        % number of lidar msmts over the horizon
        obj.n_M= estimator.n_k + sum( obj.n_ph(1:obj.M - 1) );

        % number of landmarks over the horizon
        obj.n_L_M= obj.n_M / params.m_F;
        estimator.n_L_M= obj.n_L_M;
    end
    
    % compute extraction vector
    alpha= obj.build_state_of_interest_extraction_matrix(params, estimator.XX);
    
    % number of GPS msmts over the horizon
    obj.n_M_gps= estimator.n_gps_k + sum( obj.n_gps_ph(1:obj.M - 1) );
    
    % total number of msmts (prior + relative + abs)
    obj.n_total= obj.n_M + obj.n_M_gps + (obj.M + 1) * (9); %Osama params.m
    
    % number of states to estimate
    obj.m_M= (obj.M + 1) * (9); %Osama params.m
    
    % compute the whiten Jacobian matrix A
    obj.compute_whiten_jacobian_A(estimator, params);
    
    % construct the information matrix
    obj.Gamma_fg= obj.A' * obj.A;
    
    % full covarince matrix
    obj.PX_M= inv(obj.Gamma_fg);
    
    % extract covarince matrix at time k
    estimator.PX= obj.PX_M( end - 9 + 1 : end, end - 9 + 1 : end );  %Osama params.m
    
    % find the prior covarince matrix for time k+1
    obj.PX_prior= obj.PX_M( 9 + 1 : 2*9, 9 + 1 : 2*9 );  %Osama params.m
    obj.Gamma_prior= inv(obj.PX_prior);
    
    % fault probability of each association in the preceding horizon
    obj.P_F_M= ones(obj.n_L_M + (obj.n_M_gps/6) , 1) * params.P_UA;
    
    % compute the hypotheses (n_H, n_max, inds_H)
    obj.compute_hypotheses(params)
    
    % initialization of p_hmi
    obj.p_hmi=0;
    if ( obj.n_M + obj.n_M_gps ) < ( params.m - 12 + obj.n_max*params.m_F )
        % if we don't have enough landmarks --> P(HMI)= 1
        obj.p_hmi= 1;
        
    else % we have enough msmts

        % standard deviation in the state of interest
        obj.sigma_hat= sqrt( (alpha' * obj.PX_M) * alpha );
        
        % initializing P_H vector
        obj.P_H= ones(obj.n_H, 1) * inf;
        
        for i= 0:obj.n_H
            
            % compute P(HMI | H) for the worst-case fault
            p_hmi_H= obj.compute_p_hmi_H(alpha, i, params);
            
            % Add P(HMI | H) to the integrity risk
            if i == 0
                obj.P_H_0= prod( 1 - obj.P_F_M );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H_0;
            else
                obj.P_H(i)= prod( obj.P_F_M( obj.inds_H{i} ) );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H(i);
            end
        end
    end
    
    % store integrity related data
    data.store_integrity_data(obj, estimator, counters, params) 
end

% update the preceding horizon
update_preceding_horizon(obj, estimator)

end