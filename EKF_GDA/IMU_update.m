
function x= IMU_update(x, u, g_N, tau, dt)

% Create variables (for clarity)
v= x(4:6);
phi= x(7); theta= x(8); psi= x(9);
b_f= x(10:12);
b_w= x(13:15);
f= u(1:3); 
w= u(4:6);

% Calculate paramters
R_NB= R_NB_rot(phi,theta,psi);
E_BE= invQ_BE_fn(phi,theta);

% Fill x_dot
r_dot= v;
v_dot= R_NB * ( f - b_f ) + g_N;
E_dot= E_BE * ( w - b_w );
b_f_dot= -eye(3) / tau * b_f;
b_w_dot= -eye(3) / tau * b_w;
x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];

% Return new pose
x= x + dt*x_dot;






