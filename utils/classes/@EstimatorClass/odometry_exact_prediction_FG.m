function odometry_exact_prediction_FG(obj, params)


vel= params.velocity_FG; % linear velocity of the car
phi= obj.steering_angle; % steering angle


% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
            obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
            pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_FG )];

% compute variables
s= sin(phi + obj.x_true(3)); 
c= cos(phi + obj.x_true(3));
vts= vel*params.dt_sim * s; 
vtc= vel*params.dt_sim * c;

% state evolution model jacobian
obj.Phi_k= [1 0 -vts;
          0 1  vtc;
          0 0 1];

% controls jacobian (only steering angle and wheel velocity)
Gu= [params.dt_sim * c,                       -vts;
     params.dt_sim * s,                        vtc;
     params.dt_sim * sin(phi)/params.wheelbase_FG, vel*params.dt_sim * cos(phi)/params.wheelbase_FG];

% projection of controls uncertainty on the state (only steering angle and wheel velocity)
obj.D_bar= Gu * params.W_odometry_FG * Gu';

end