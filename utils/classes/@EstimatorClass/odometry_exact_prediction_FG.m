function odometry_exact_prediction_FG(obj, params)


vel= params.velocity_FG; % linear velocity of the car
phi= obj.steering_angle; % steering angle


% True State
obj.XX= [obj.XX(1) + vel * params.dt_sim * cos(phi + obj.XX(3));
            obj.XX(2) + vel * params.dt_sim * sin(phi + obj.XX(3));
            pi_to_pi( obj.XX(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_FG )];

% Add noise to the controls
%Vn= vel + normrnd(0, params.sig_velocity_sim); 
%Gn= phi + normrnd(0, params.sig_steering_angle_sim); 

% compute variables
s= sin(phi + obj.XX(3)); 
c= cos(phi + obj.XX(3));
vts= vel*params.dt_sim * s; 
vtc= vel*params.dt_sim * c;

% jacobians   
obj.Phi_k= [1 0 -vts;
          0 1  vtc;
          0 0 1];
      
Gu= [params.dt_sim * c,                       -vts;
     params.dt_sim * s,                        vtc;
     params.dt_sim * sin(phi)/params.wheelbase_FG, vel*params.dt_sim * cos(phi)/params.wheelbase_FG];

obj.D_bar= Gu * params.W_odometry_FG * Gu';
% predict covariance
% obj.PX= obj.Phi_k * obj.PX * obj.Phi_k' + obj.Gu * params.W_odometry_sim * obj.Gu';

end