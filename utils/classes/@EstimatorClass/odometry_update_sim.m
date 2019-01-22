function odometry_update_sim(obj, params)


vel= params.velocity_sim;
phi= params.steering_angle_sim;

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
            obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
            pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

% Add noise to the controls
Vn= vel + normrnd(0, params.sig_velocity_sim); 
Gn= phi + normrnd(0, params.sig_steering_angle_sim); 

% compute variables
s= sin(Gn + obj.XX(3)); 
c= cos(Gn + obj.XX(3));
vts= Vn*params.dt_sim * s; 
vtc= Vn*params.dt_sim * c;

% Predict state
obj.XX= [obj.XX(1) + vtc;
         obj.XX(2) + vts;
         pi_to_pi( obj.XX(3)+ Vn*params.dt_sim * sin(Gn) / params.wheelbase_sim )];

% jacobians   
obj.Phi_k= [1 0 -vts;
          0 1  vtc;
          0 0 1];
      
Gu= [params.dt_sim * c,                       -vts;
     params.dt_sim * s,                        vtc;
     params.dt_sim * sin(Gn)/params.wheelbase_sim, Vn*params.dt_sim * cos(Gn)/params.wheelbase_sim];

obj.D_bar= Gu * params.W_odometry_sim * Gu';
 
% predict covariance
obj.PX= obj.Phi_k * obj.PX * obj.Phi_k' + obj.D_bar;

end