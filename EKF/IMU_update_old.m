
function [x]= IMU_update(x,u,G,tau,dt)

% Calculate paramters
v= x(4:6);
bias= x(10:15);
f_B= u(1:3); wib_B= u(4:6);
f_B= f_B - bias(1:3);
wib_B= wib_B - bias(4:6);
phi= x(7); theta= x(8); psi= x(9);
R_NB= R_NB_rot(phi,theta,psi);
invQ_BE= invQ_BE_fn(phi,theta);

% Fill x_dot
r_dot= v;
v_dot= R_NB*f_B + G;
E_dot= invQ_BE*(wib_B);
f_bias_dot= -eye(6) / tau * bias;
x_dot= [r_dot; v_dot; E_dot; f_bias_dot];

% Return new pose
x= x + dt*x_dot;






