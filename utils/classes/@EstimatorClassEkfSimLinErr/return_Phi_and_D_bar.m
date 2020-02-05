
function [Phi, D_bar, D_bar_ub]= return_Phi_and_D_bar(obj, x, vel, phi, params)
% this function computes the state evolution matrix and its noise at the
% corresponding time where the estimate is x and the odometry inputs are
% vel and phi


% compute variables
s= sin(phi + x(params.ind_yaw));
c= cos(phi + x(params.ind_yaw));
vts= vel*params.dt_sim * s;
vtc= vel*params.dt_sim * c;

% state evolution model jacobian
Phi= [1 0 -vts;
      0 1  vtc;
      0 0 1];

% state evolution model hessian
P_half = sqrtm(obj.PX);
H = [(vel*params.dt_sim*(P_half(1,3)^2+P_half(2,3)^2+P_half(3,3)^2))^2 0 0;
     0 (vel*params.dt_sim*(P_half(1,3)^2+P_half(2,3)^2+P_half(3,3)^2))^2 0;
     0 0 0];

% controls jacobian (only steering angle and wheel velocity)
Gu= [params.dt_sim * c,                       -vts;
     params.dt_sim * s,                        vtc;
     params.dt_sim * sin(phi)/params.wheelbase_sim, vel*params.dt_sim * cos(phi)/params.wheelbase_sim];

% controls hessian
Hc= [(params.W_odometry_sim(2,2)*vel*params.dt_sim)^2 0 0;
     0 (params.W_odometry_sim(2,2)*vel*params.dt_sim)^2 0;
     0 0 (params.W_odometry_sim(2,2)*vel*params.dt_sim/params.wheelbase_sim)^2];

% projection of controls uncertainty on the state (only steering angle and wheel velocity)
D_bar_ub= Gu * params.W_odometry_sim * Gu' + (obj.Dc_cov_controller/4).*Hc + (obj.Dc_cov_state/4).*H;
D_bar= Gu * params.W_odometry_sim * Gu';

end
