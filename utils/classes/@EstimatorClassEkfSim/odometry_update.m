function odometry_update(obj, params)

% velocity & steering angle
vel= params.velocity_sim;
phi= params.steering_angle_sim;

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
             obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
             pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

if params.SWITCH_OFFLINE    
     [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.x_true, vel, phi, params);
     
else

    % Add noise to the controls
    vel= vel + normrnd(0, params.sig_velocity_sim);
    phi= phi + normrnd(0, params.sig_steering_angle_sim);

    % compute state evolution matrix and its noise covariance matrix
    [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.XX, vel, phi, params);
    
    obj.XX= obj.return_odometry_update(obj.XX, [vel; phi], params);
    
end


% in factor graphs the covariance is calculated after the update alltogether
if ~params.SWITCH_FACTOR_GRAPHS
    % predict covariance
    obj.PX= obj.Phi_k * obj.PX * obj.Phi_k' + obj.D_bar;
end


% save the velocity and steering angle 
obj.odometry_k= [vel; phi];

end
