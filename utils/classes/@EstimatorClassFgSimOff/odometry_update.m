
function odometry_update_sim_fg(obj, params)
% this function updates the estimate and true state for the given odometry
% (velocity, steering angle) controls. Note that the the estimate is
% updated with the computed controls, i.e. the ones we want to send to the
% system. The true state is udpated with the actual controls executed in
% the system, which have noise.


% velocity & steering angle
vel= params.velocity_sim;
phi= obj.steering_angle;

% if it's the offline im analysis --> compute matrices and update true x
if params.SWITCH_OFFLINE
    % compute state evolution matrix and its noise covariance matrix
    [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.x_true, vel, phi, params);
    
% if we are online --> add noise
else  
    % compute state evolution matrix and its noise covariance matrix
    [obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.XX, vel, phi, params);
    
    % estimate state with computed controls
    obj.XX= obj.return_odometry_update_sim(obj.XX, [vel; phi], params);
    
    % Add noise to the computed controls
    vel= vel + normrnd(0, params.sig_velocity_sim);
    phi= phi + normrnd(0, params.sig_steering_angle_sim);
    
end

% True State
obj.x_true= obj.return_odometry_update_sim(obj.x_true, [vel; phi], params);

% save the velocity and steering angle 
obj.odometry_k= [vel; phi];

end
