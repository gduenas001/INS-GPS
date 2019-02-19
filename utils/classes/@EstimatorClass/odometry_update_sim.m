function odometry_update_sim(obj, params)

% velocity & steering angle
vel= params.velocity_sim;
phi= params.steering_angle_sim;

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
            obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
            pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

if params.SWITCH_FACTOR_GRAPHS
    
    % compute variables
    s= sin(phi + obj.x_true(3));
    c= cos(phi + obj.x_true(3));
    vts= vel*params.dt_sim * s;
    vtc= vel*params.dt_sim * c;
    
else

    % Add noise to the controls
    vel= vel + normrnd(0, params.sig_velocity_sim);
    phi= phi + normrnd(0, params.sig_steering_angle_sim);
    
    % compute variables
    s= sin(phi + obj.XX(3));
    c= cos(phi + obj.XX(3));
    vts= vel*params.dt_sim * s;
    vtc= vel*params.dt_sim * c;
    
    % Predict state
    obj.XX= [obj.XX(1) + vtc;
             obj.XX(2) + vts;
             pi_to_pi( obj.XX(3)+ vel*params.dt_sim * sin(phi) / params.wheelbase_sim )];
    
end

% state evolution model jacobian
obj.Phi_k= [1 0 -vts;
            0 1  vtc;
            0 0 1];

% controls jacobian (only steering angle and wheel velocity)
Gu= [params.dt_sim * c,                       -vts;
     params.dt_sim * s,                        vtc;
     params.dt_sim * sin(phi)/params.wheelbase_sim, vel*params.dt_sim * cos(phi)/params.wheelbase_sim];

% projection of controls uncertainty on the state (only steering angle and wheel velocity)
obj.D_bar= Gu * params.W_odometry_sim * Gu';

% in factor graphs the covariance is calculated after the update alltogether
if ~params.SWITCH_FACTOR_GRAPHS
    % predict covariance
    obj.PX= obj.Phi_k * obj.PX * obj.Phi_k' + obj.D_bar;
end

end