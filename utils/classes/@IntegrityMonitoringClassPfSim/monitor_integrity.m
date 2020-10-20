
function monitor_integrity(obj, estimator, counters, data,  params)


% calculate the current number of LMs in PH; only before starting integrity monitoring
%if params.SWITCH_FIXED_LM_SIZE_PH && isempty(obj.p_hmi)
%    
%    % current horizon measurements
%    obj.n_M= sum( obj.n_ph(1:obj.M) ) + estimator.n_k;
%    % current horizon LMs
%    obj.n_L_M= obj.n_M / params.m_F;
%    estimator.n_L_M= obj.n_L_M;
%    % update the length of PH
%    obj.M= obj.M +1; 
%    
%end

% monitor integrity if the number of LMs in the preceding horizon is more than threshold
%if  ( params.SWITCH_FIXED_LM_SIZE_PH &&...
%    obj.n_L_M >= params.min_n_L_M ) ||...
%    ( ~params.SWITCH_FIXED_LM_SIZE_PH && counters.k_im > obj.M )

%    % Modify preceding horizon to have enough landmarks
%    if params.SWITCH_FIXED_LM_SIZE_PH
        
%        obj.compute_required_epochs_for_min_LMs(params, estimator)
        
%    else
        
%        % number of absolute msmts over the horizon
%        obj.n_M= estimator.n_k + sum( obj.n_ph(1:obj.M - 1) );

%        % number of landmarks over the horizon
%        obj.n_L_M= obj.n_M / params.m_F;
%        estimator.n_L_M= obj.n_L_M;
%    end
    
%    % compute extraction vector
    obj.build_state_of_interest_extraction_matrix(params, estimator.XX_update);
       
%    % total number of msmts (prior + relative + abs)
%    obj.n_total= obj.n_M + (obj.M + 1) * (params.m);
    
%    % number of states to estimate
%    obj.m_M= (obj.M + 1) * params.m;
    
%    % compute the H whiten Jacobian A
%    obj.compute_whiten_jacobian_A(estimator, params);% TODO: use the new function, remove this one
    
%    % construct the information matrix
%    obj.Gamma_fg= obj.A' * obj.A;
    
%    % full covarince matrix 
%     TODO: use shur complement to ge the covariance
%    obj.PX_M= inv(obj.Gamma_fg);
    
%    % extract covarince matrix at time k
%    estimator.PX= obj.PX_M( end - params.m + 1 : end, end - params.m + 1 : end );
    
%    % find the prior covarince matrix for time k+1
%    obj.PX_prior= obj.PX_M( params.m + 1 : 2*params.m, params.m + 1 : 2*params.m );
%    obj.Gamma_prior= inv(obj.PX_prior);
    
    % Detector parameters
    %obj.N_k_predict = length(estimator.particles_indices_predict);
    %Unique_particles_indices_predict = unique(estimator.particles_indices_predict);
    %obj.M_k_predict = length(Unique_particles_indices_predict);
    %obj.N_k_predict = length(estimator.particles_indices_predict);
    %obj.N_r_2_predict = 0;
    %for i = 1 : obj.M_k_predict
    %    obj.N_r_2_predict = obj.N_r_2_predict + ( sum( estimator.particles_indices_predict == Unique_particles_indices_predict(i) ) )^2;
    %end
    
    %obj.N_k_prior = length(estimator.particles_indices_prior);
    obj.N_k_predict = length(estimator.particles_indices_predict);
    %obj.Unique_particles_indices_prior = unique(estimator.particles_indices_prior);
    %obj.M_k_prior = length(obj.Unique_particles_indices_prior);
    %obj.N_k_prior = length(estimator.particles_indices_prior);
    
    %obj.N_r_2_prior = 0;
    obj.Y_k=estimator.V_k;
    %obj.H_G_particles=inf*ones(estimator.n_k,params.m*obj.M_k_prior);
    Sum_H_F = 0;
    %for i = 1 : obj.M_k_prior
    for i = 1 : obj.N_k_predict
        %N_r_2_prior_i= ( sum( estimator.particles_indices_prior == obj.Unique_particles_indices_prior(i) ) )^2;
        %obj.N_r_2_prior = obj.N_r_2_prior + N_r_2_prior_i;
        %ind= find(estimator.particles_indices_prior == obj.Unique_particles_indices_prior(i),1);
        %H_G_i = estimator.H_k_particles(:,params.m*(ind-1)+1:params.m*ind)*estimator.G_k_particles(:,params.m*(ind-1)+1:params.m*ind);
        %obj.Y_k = obj.Y_k + ( N_r_2_prior_i/(obj.N_k_prior^2) * H_G_i * estimator.SX_prior * H_G_i');
        obj.Y_k = obj.Y_k + ( 1/(obj.N_k_predict^2) * estimator.H_k_particles(:,params.m*(i-1)+1:params.m*i) * estimator.SX_predict * estimator.H_k_particles(:,params.m*(i-1)+1:params.m*i)');
        %Sum_H_F = Sum_H_F + sqrt(N_r_2_prior_i)*estimator.H_k_particles(:,params.m*(ind-1)+1:params.m*ind)*estimator.F_k_particles(params.m*(ind-1)+1:params.m*ind,:);
    end
    %obj.U_k = Sum_H_F *params.W_odometry_sim* Sum_H_F'/(obj.N_k_prior^2);
    %obj.Y_k = obj.Y_k +obj.U_k;
    
    % Estimate Error paramters
    obj.N_k_update = length(estimator.particles_indices_update);
    %Unique_particles_indices_update = unique(estimator.particles_indices_update);
    %obj.M_k_update = length(Unique_particles_indices_update);
    %obj.N_k_update = length(estimator.particles_indices_update);
    %obj.N_r_2_update = 0;
    %for i = 1 : obj.M_k_update
    %    obj.N_r_2_update = obj.N_r_2_update + ( sum( estimator.particles_indices_update == Unique_particles_indices_update(i) ) )^2;
    %end
    
    % set detector threshold from the continuity req
    %obj.T_d = finv(1 - obj.C_req,estimator.n_k,obj.M_k_prior-params.m);
    obj.T_d = finv(1 - obj.C_req,estimator.n_k,obj.N_k_predict-params.m);
    
    %obj.Y_k = estimator.H_k*( obj.N_r_2_predict*estimator.SX_predict/(obj.N_k_predict^2) )*estimator.H_k'+estimator.V_k;
    inv_Y_k = inv(obj.Y_k);
    inv_V_k = inv(estimator.V_k);
    % Detector
    %obj.q_k = (obj.M_k_predict-params.m)/(estimator.n_k * (obj.M_k_predict-1)) * (estimator.z_k-estimator.h_k)'*inv_Y_k*(estimator.z_k-estimator.h_k);
    obj.q_k = (obj.M_k_prior-params.m)/(estimator.n_k * (obj.M_k_prior-1)) * ( estimator.z_k - transpose(mean(estimator.h_k_i')) )' * inv_Y_k * ( estimator.z_k - transpose(mean(estimator.h_k_i')) );

% %     obj.beta_k = max(eig( sqrtm(inv_Y_k)*(obj.Y_k*inv_V_k)*obj.Y_k*sqrtm(inv_Y_k) ));
% %     
% %     obj.eta_k_min = 0;
% %     for i=1:obj.N_k_prior
% %         %obj.eta_k_min = obj.eta_k_min + (2*pi)^(-0.5*estimator.n_k) * det(estimator.V_k)^(-0.5) * exp( -0.5*obj.beta_k*( sqrt(estimator.n_k * (obj.M_k_prior-1) * obj.T_d /(obj.M_k_prior-params.m) )+ sqrt( (transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i))'*inv_Y_k*(transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i)) ) )^2 );
% %         obj.eta_k_min = obj.eta_k_min + (2*pi)^(-0.5*estimator.n_k) * det(estimator.V_k)^(-0.5) * exp( -0.5*( sqrt(obj.beta_k*estimator.n_k * (obj.M_k_prior-1) * obj.T_d /(obj.M_k_prior-params.m) )+ sqrt( (transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i))'*inv_V_k*(transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i)) ) )^2 );
% %     end
% %     
% %     obj.eta_k_max = 0;
% %     for i=1:obj.N_k_prior
% %         %obj.eta_k_hat = obj.eta_k_hat + (2*pi)^(-0.5*estimator.n_k) * det(estimator.V_k)^(-0.5) * exp( -0.5*obj.beta_k*( sqrt(estimator.n_k * (obj.M_k_prior-1) * obj.T_d /(obj.M_k_prior-params.m) )+ sqrt( (transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i))'*inv_Y_k*(transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i)) ) )^2 );
% %         obj.eta_k_max = obj.eta_k_max + (2*pi)^(-0.5*estimator.n_k) * det(estimator.V_k)^(-0.5) * exp( -0.5*( max(0,sqrt( (transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i))'*inv_V_k*(transpose(mean(estimator.h_k_i'))-estimator.h_k_i(:, i)) )-sqrt(obj.beta_k*estimator.n_k * (obj.M_k_prior-1) * obj.T_d /(obj.M_k_prior-params.m) )) )^2 );
% %     end
    
    
    %if obj.eta_k_min < 1e-90
    %    obj.eta_k_min = 1e-90;
    %end
    %    % set detector threshold from the continuity req
    % obj.T_d= chi2inv( 1 - obj.C_req, obj.n_M );
    
    % obj.prob_of_MA(estimator, params);
    
    %obj.P_MA_M = [ obj.P_MA_k ; cell2mat(obj.P_MA_ph(1:obj.M-1)') ];
    obj.n_L_k = estimator.n_k / params.m_F;
    
    % fault probability of each association in the preceding horizon
    obj.P_F_M= ones(obj.n_L_k, 1) * params.P_UA;% + obj.P_MA_M;
    
    % compute the hypotheses (n_H, n_max, inds_H)
    obj.compute_hypotheses(params)
    
    % tic
    
    % initialization of p_hmi
    obj.p_hmi=0;
    % obj.f_avg=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %if obj.n_M < params.m + obj.n_max*params.m_F
        % if we don't have enough landmarks --> P(HMI)= 1
    %    obj.p_hmi= 1;
        
    %else % we have enough msmts
        
        % Least squares residual matrix
        %obj.M_M= eye( obj.n_total ) - (obj.A / (obj.A'*obj.A)) * obj.A';
        
        % standard deviation in the state of interest
        %obj.sigma_hat= sqrt( (alpha' / obj.Gamma_fg) * alpha );
        
    % initializing P_H vector
    obj.P_H= ones(obj.n_H, 1) * inf;

    for i= 0:obj.n_H

        % compute P(HMI | H) for the worst-case fault
        % if (obj.M_k_update==1)||(obj.M_k_prior<=params.m)
        %     p_hmi_H= 1;
        % else
        
        p_hmi_H= obj.compute_p_hmi_H(i, params, estimator);

        if i==0
            p_hmi_0 = p_hmi_H;
        elseif p_hmi_H < p_hmi_0
            p_hmi_H = p_hmi_0;
        end
        % end
        %p_hmi_H = 1;
        %obj.f_avg=obj.f_avg+obj.f_M_mag_out;
        % Add P(HMI | H) to the integrity risk
        if i == 0
            obj.P_H_0= prod( 1 - obj.P_F_M );
            obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H_0;
        else
            obj.P_H(i)= prod( obj.P_F_M( obj.inds_H{i} ) );
            obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H(i);
        end
    end
    %if ~params.SWITCH_ONLY_ONE_LM_FAULT
    %    obj.p_hmi = obj.p_hmi + params.I_H;
    %end
        %obj.f_avg=obj.f_avg/(obj.n_H+1);
    %end
    
    %obj.p_hmi_elapsed_time=toc;
    
    % store integrity related data
    data.store_integrity_data(obj, estimator, counters, params)
    
%else
    
    %obj.prob_of_MA(estimator, params);
    
%end

% update the preceding horizon
%update_preceding_horizon(obj, estimator)

end