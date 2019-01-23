function prob_of_MA(obj, estimator, params)

if isempty(obj.A_M) == 1
    
    mu_k = 0;
    
else
    
    Q = obj.A_M' * obj.Phi_ph{1}' * estimator.PX([1:2,9],[1:2,9]) * obj.Phi_ph{1} * obj.A_M;
    lambda_max_Q = max( eig( Q ) );
    mu_k = lambda_max_Q * ( sqrt(obj.T_d) - sqrt( chi2inv( 1 - obj.I_MA , obj.n_M ) ) )^2;
    
end

spsi= sin(estimator.XX(9));
cpsi= cos(estimator.XX(9));
h_t = zeros(2,1);
h_l = zeros(2,1);
obj.P_MA_k = inf * ones(size(estimator.associated_landmarks_at_k));
for t=1:size(estimator.associated_landmarks_at_k , 1)
    P_MA_k_t = size(estimator.FoV_landmarks_at_k, 1) - 1;
    landmark= estimator.landmark_map( estimator.associated_landmarks_at_k(t) ,: );
    dx= landmark(1) - estimator.XX(1);
    dy= landmark(2) - estimator.XX(2);
    h_t(1)=  dx*cpsi + dy*spsi;
    h_t(2)= -dx*spsi + dy*cpsi;
    for l=1:size(estimator.FoV_landmarks_at_k, 1)
        if (l ~= t)
            landmark= estimator.landmark_map( estimator.FoV_landmarks_at_k(l) ,: );
            dx= landmark(1) - estimator.XX(1);
            dy= landmark(2) - estimator.XX(2);
            h_l(1)=  dx*cpsi + dy*spsi;
            h_l(2)= -dx*spsi + dy*cpsi;
            H_l= [-cpsi, -spsi, -dx*spsi + dy*cpsi; spsi, -cpsi, -dx*cpsi - dy*spsi ];
            Y_l= H_l * estimator.PX([1:2,9],[1:2,9]) * H_l' + params.R_lidar;
            inv_Y_l = inv(Y_l);
            y_l_t = h_l - h_t;
            MIIN_l_t = sqrt( y_l_t' * inv_Y_l * y_l_t );
            chi_dof = obj.m + params.m_F;
            P_MA_k_t = P_MA_k_t - ncx2cdf( (MIIN_l_t-params.T_NN)^2 , chi_dof, mu_k );
        end
    end
    obj.P_MA_k(t) = P_MA_k_t;
end

end