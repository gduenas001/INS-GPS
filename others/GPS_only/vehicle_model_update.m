
function [x,P] = vehicle_model_update(x, P, dt)

Phi= [eye(3), dt*eye(3), zeros(3,9);
      zeros(3), eye(3), zeros(3,9);
      zeros(9,15)];
  
x= Phi*x;

sig_w_pos= 0.01; % 10cm
sig_w_vel= 0.1; % 1m/s
sig_w_phi= deg2rad(0.2); % 2deg
sig_w_theta= deg2rad(0.2); % 2deg
sig_w_psi= deg2rad(0.8); % 8deg
W= diag( [sig_w_pos, sig_w_pos, sig_w_pos, sig_w_vel, sig_w_vel, sig_w_vel, ...
         sig_w_phi, sig_w_theta, sig_w_psi, zeros(1,6)] ).^2; 

P= Phi*P*Phi' + W;



















