
function x= IMU_update( x, u, g_N, taua, tauw, dt )

% Create variables (for clarity)
v= x(4:6);
phi= x(7); theta= x(8); psi= x(9);
b_f= x(10:12);
b_w= x(13:15);
f= u(1:3); 
w= u(4:6);

% Calculate parameters
R_NB= R_NB_rot(phi,theta,psi);
Q_BE= Q_BE_fn(phi,theta);

% % % % Compute the Euler angles
% % % E_dot= Q_BE * ( w - b_w );
% % % E= x(7:9) + E_dot*dt;
% % % phi= E(1); theta= E(2); psi= E(3);
% % % x(7:9)= E;
% % % 
% % % % Rotate the acceleration
% % % R_NB= R_NB_rot(phi,theta,psi);
% % % acc= (R_NB * ( f - b_f ) + g_N);
% % % 
% % % % Compute the velocity
% % % x(4:6)= x(4:6) + acc*dt;
% % % 
% % % % Compute the new velocity
% % % x(1:3)= x(1:3) + dt*v + 0.5*acc*dt^2;
% % % 
% % % % compute the new biases
% % % b_f_dot= -eye(3) / taua * b_f;
% % % b_w_dot= -eye(3) / tauw * b_w;
% % % b_dot= [b_f_dot; b_w_dot];
% % % x(10:15)= x(10:15) + b_dot*dt;

r_dot= v;
v_dot= R_NB * ( f - b_f ) + g_N;
E_dot= Q_BE * ( w - b_w );
b_f_dot= -eye(3) / taua * b_f;
b_w_dot= -eye(3) / tauw * b_w;
x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];

% Return new pose
x= x + dt*x_dot;


% x(9)= pi_to_pi(x(9));




