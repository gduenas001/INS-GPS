
function p_hmi_H= compute_p_hmi_H(obj, alpha, fault_ind, params)

 
% build extraction matrix
if fault_ind == 0
    % obj.compute_E_matrix_fg( 0, params.m_F);
    obj.compute_B_matrix_fg( [] , params.m_F );
else
    % obj.compute_E_matrix_fg( obj.inds_H{fault_ind}, params.m_F);
    obj.compute_B_matrix_fg( obj.inds_H{fault_ind} , params.m_F );
end

obj.Gamma_fg_j= obj.A' * obj.B_j' * obj.B_j * obj.A;

obj.sigma_hat_j = sqrt( alpha' / obj.Gamma_fg_j * alpha );

obj.sigma_hat_delta_j= sqrt( obj.sigma_hat_j^2 - obj.sigma_hat^2 );

obj.T_delta_j= norminv( 1 - obj.C_req/(2*obj.n_H) ) * obj.sigma_hat_delta_j;

if ( ( (params.alert_limit-obj.T_delta_j)>=0 ) && isreal(obj.sigma_hat_j) && isreal(obj.T_delta_j) )
    p_hmi_H= 2*normcdf( -1*(params.alert_limit-obj.T_delta_j), 0, obj.sigma_hat_j );
else
    p_hmi_H= 1;
end

end
