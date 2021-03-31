function odometry_update(obj, params)

% velocity & steering angle
vel= params.velocity_sim;
phi= obj.steering_angle;
% phi= params.steering_angle_sim;

if params.SWITCH_OFFLINE    
     [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.x_true, vel, phi, params);
     
else

    % compute state evolution matrix and its noise covariance matrix
    [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.XX(1:params.m), vel, phi, params);
    
    obj.XX(1:params.m)= obj.return_odometry_update(obj.XX(1:params.m), [vel; phi], params);
    
end

% save the velocity and steering angle 
obj.odometry_k= [vel; phi];

% Add noise to the controls
vel= vel + normrnd(0, params.sig_velocity_sim);
phi= phi + normrnd(0, params.sig_steering_angle_sim);

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
             obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
             pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

% in factor graphs the covariance is calculated after the update alltogether
if ~params.SWITCH_FACTOR_GRAPHS
    % predict covariance
    Phi_k= [obj.Phi_k, zeros(params.m,length(obj.XX)-params.m);
            zeros(length(obj.XX)-params.m,params.m), eye(length(obj.XX)-params.m)];
    obj.PX_prediction= Phi_k * obj.PX_update * Phi_k' + [obj.D_bar, zeros(params.m,length(obj.XX)-params.m); zeros(length(obj.XX)-params.m,length(obj.XX))];
end

end
