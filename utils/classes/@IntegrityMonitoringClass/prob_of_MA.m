function prob_of_MA(obj, estimator, association, params)

% TODO: for distinction between test and simulation
if params.SWITCH_SIM
    ind_yaw= 3;
    ind_pose= 1:3;
else
    ind_yaw= 9;
    ind_pose= [1:2,3];
end


if isempty(obj.A_M)
    mu_k = 0;
else
    % compute Q matrix with A_M_(k-1) , Phi_(k-1), P_k, n_M_(k-1)
    Q= obj.A_M' * obj.Phi_ph{1}' * estimator.PX(ind_pose, ind_pose) * obj.Phi_ph{1} * obj.A_M;
    lambda_max_Q= max( eig( Q ) );
    mu_k= lambda_max_Q * ( sqrt(obj.T_d) - sqrt( chi2inv(1 - params.I_MA , obj.n_M) ) )^2;
end

% dof of the non-central chi-square in the P(MA)
chi_dof= obj.m + params.m_F;

% allocate memory
spsi= sin(estimator.XX(ind_yaw));
cpsi= cos(estimator.XX(ind_yaw));
h_t= zeros(2,1);
h_l= zeros(2,1);
association_of_associated_features= association( association ~= 0);
obj.P_MA_k= ones(size(association_of_associated_features)) * (-1);

% loop through each associated landmark
for t= 1:length(association_of_associated_features)
    % take the landmark ID
    lm_id_t= association_of_associated_features(t);
    
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
            Y_l= H_l * estimator.PX(ind_pose, ind_pose) * H_l' + params.R_lidar;
            
            % individual innovation norm between landmarks l and t
            IIN_l_t= sqrt( y_l_t' / Y_l * y_l_t );
            
            % if one of the landmarks is too close to ensure P(MA) --> set to one
            if IIN_l_t < sqrt(params.T_NN)
                obj.P_MA_k(t)= 1;
                break
            else
                obj.P_MA_k(t)= obj.P_MA_k(t) -...
                    ncx2cdf( ( IIN_l_t - sqrt(params.T_NN) )^2 , chi_dof, mu_k );
            end
        end
    end
end

end