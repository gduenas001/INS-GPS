function compute_imu_Phi_k( obj, params, FG, epoch )
    
    % linearize and discretize IMU model
    obj.linearize_discretize(FG.imu{epoch}, params.dt_imu, params)
    
    % process noise cov matrix beween two lidar epochs
    D_bar_init= obj.D_bar;
    for i = 1:11
        obj.D_bar= obj.Phi_k*obj.D_bar*obj.Phi_k' + D_bar_init;
    end
    % imu jacobian matrix beween two lidar epochs
    obj.Phi_k= obj.Phi_k^12;
    
    %remove imu biases
    obj.Phi_k= obj.Phi_k(1:9,1:9); %Osama
    obj.D_bar= obj.D_bar(1:9,1:9); %Osama
    
end