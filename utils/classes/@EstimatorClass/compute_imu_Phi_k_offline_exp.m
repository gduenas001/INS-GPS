function compute_imu_Phi_k_offline_exp( obj, params, FG, epoch )
    
    % linearize and discretize IMU model
    obj.linearize_discretize(FG.imu{epoch}, params.dt_imu, params)
    
    % whiten jacobian for imu
    obj.Phi_k= sqrtm( inv(obj.D_bar) ) * obj.Phi_k;
    
end