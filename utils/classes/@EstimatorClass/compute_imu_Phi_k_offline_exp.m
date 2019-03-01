function compute_imu_Phi_k_offline_exp( obj, params, FG, epoch )
    
    % linearize and discretize IMU model
    obj.linearize_discretize(FG.imu{epoch}, params.dt_imu, params)
    
    % whiten jacobian for imu
    D_bar_init= obj.D_bar;
    for i = 1:11
        obj.D_bar= obj.Phi_k*obj.D_bar*obj.Phi_k' + D_bar_init;
    end
    obj.Phi_k= obj.Phi_k^12;
end