function prob_of_MA(obj, estimator, params)


% dof of the non-central chi-square in the P(MA)
chi_dof= obj.m + params.m_F;

% allocate memory
spsi= sin(estimator.XX(params.ind_yaw));
cpsi= cos(estimator.XX(params.ind_yaw));
h_t= zeros(2,1);
h_l= zeros(2,1);
estimator.association_no_zeros= estimator.association( estimator.association ~= 0);
obj.P_MA_k= ones(size(estimator.association_no_zeros)) * (-1);


% compute kappa
if isempty(obj.A_M)
    obj.mu_k = 0;
elseif obj.n_L_M - obj.n_max < 2
    obj.kappa= 1;
    obj.mu_k= obj.kappa * ( sqrt(obj.T_d) - sqrt( chi2inv(1 - params.I_MA , obj.n_M) ) )^2;
else
    % compute Q matrix with A_M_(k-1) , Phi_(k-1), P_k, n_M_(k-1)
    Q= obj.A_M' * obj.Phi_ph{1}' * estimator.PX(params.ind_pose, params.ind_pose) * obj.Phi_ph{1} * obj.A_M;
    
    obj.kappa= 0;
    C = nchoosek(1:obj.n_L_M,obj.n_max);%set of possible fault indices for n_max simultanous faults
    for i= 1:size(C,1)
        % build extraction matrix
        obj.compute_E_matrix(C(i,:), params.m_F);
        kappa_H= eigs( obj.E*Q*obj.E', 1) * eigs( obj.E*obj.M_M*obj.E', 1, 'smallestabs');
        % take the largest kappa
        if kappa_H > obj.kappa, obj.kappa= kappa_H; end
    end
    obj.mu_k= obj.kappa * ( sqrt(obj.T_d) - sqrt( chi2inv(1 - params.I_MA , obj.n_M) ) )^2;
end


% loop through each associated landmark
for t= 1:length(estimator.association_no_zeros)
    % take the landmark ID
    lm_id_t= estimator.association_no_zeros(t);
    
    % initialize the P(MA)
    obj.P_MA_k(t)= length(estimator.FoV_landmarks_at_k) - 1;
    
    % build the necessary parameters
    landmark= estimator.landmark_map( lm_id_t ,: );
    dx= landmark(1) - estimator.XX(1);
    dy= landmark(2) - estimator.XX(2);
    h_t(1)=  dx*cpsi + dy*spsi;
    h_t(2)= -dx*spsi + dy*cpsi;
    
    % loop through every possible landmark in the FoV (potential MA)
    for l= 1:length(estimator.FoV_landmarks_at_k)
        % take landmark ID
        lm_id_l= estimator.FoV_landmarks_at_k(l);
        if lm_id_t ~= lm_id_l
            % extract the landmark
            landmark= estimator.landmark_map( lm_id_l ,: );
            
            % compute necessary intermediate parameters
            dx= landmark(1) - estimator.XX(1);
            dy= landmark(2) - estimator.XX(2);
            h_l(1)=  dx*cpsi + dy*spsi;
            h_l(2)= -dx*spsi + dy*cpsi;
            H_l= [-cpsi, -spsi, -dx*spsi + dy*cpsi;...
                   spsi, -cpsi, -dx*cpsi - dy*spsi ];
            y_l_t= h_l - h_t;
            Y_l= H_l * estimator.PX(params.ind_pose, params.ind_pose) * H_l' + params.R_lidar;
            
            % individual innovation norm between landmarks l and t
            IIN_l_t= sqrt( y_l_t' / Y_l * y_l_t );
            
            % if one of the landmarks is too close to ensure P(MA) --> set to one
            if IIN_l_t < sqrt(params.T_NN)
                obj.P_MA_k(t)= 1;
                break
            else
                obj.P_MA_k(t)= obj.P_MA_k(t) -...
                    ncx2cdf( ( IIN_l_t - sqrt(params.T_NN) )^2 , chi_dof, obj.mu_k );
            end            
        end
    end
    
    % landmark selection
    if params.SWITCH_LM_SELECTION
        if obj.P_MA_k(t) > params.P_MA_max
            obj.P_MA_k(t)= -1;
            estimator.association_no_zeros(t)= -1;
            estimator.association( estimator.association == lm_id_t )= 0;
        end
    end
    
    % not more than probability one
    if obj.P_MA_k(t) > 1, obj.P_MA_k(t)= 1; end
end

% remove non-associated ones
if params.SWITCH_LM_SELECTION
    obj.P_MA_k( obj.P_MA_k == -1 )= [];
    estimator.association_no_zeros( estimator.association_no_zeros == -1 )= [];
end

end