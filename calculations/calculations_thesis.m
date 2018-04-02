
clear; clc; close all;

syms('phi', 'theta', 'psi', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'g', 'real')
syms('ax0', 'ay0', 'az0', 'wx0', 'wy0', 'wz0', 'real')
syms('phi0', 'theta0', 'psi0')

% Build vectors
f_B= [ax; ay; az]; % Accelerations measured by IMU
wib_B= [wx; wy; wz]; % Body rates measure by IMU
E= [phi; theta; psi]; % Euler angles
G= [0; 0; g]; % where g is positive

% From body rates to Euler rates
invQ_BE= [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi), -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
      
% Rotation matrices for each axis
R_psi= [cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0, 0, 1];
R_theta= [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
R_phi= [1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];

% Rotation matrices B to N and N to B
R_NB= R_psi*R_theta*R_phi;
R_BN= R_NB';

%% Euler rates
E_dot= invQ_BE * wib_B;

% Jacobians
F_E= simplify([diff(E_dot, phi) , diff(E_dot, theta), diff(E_dot, psi)]); % pretty(F_E);
F_w= simplify([diff(E_dot,wx), diff(E_dot,wy), diff(E_dot,wz)]); % pretty(F_w);

% Evaluation for initial conditions
subs(F_E, [phi,theta,wy,wz], [phi0,theta0,wy0,wz0])

%% Velocity
v_dot_N= simplify( R_NB * f_B + G );

H_E= simplify( [diff(v_dot_N, phi) , diff(v_dot_N, theta), diff(v_dot_N, psi)] );

f_Bx= [0,       f_B(3), -f_B(2);
      -f_B(3),  0,       f_B(1);
       f_B(2), -f_B(1),  0     ];
   
% The one in the thesis
H_E2= simplify(R_NB*f_Bx);

% Evaluation
symvar(H_E)
subs(H_E, [phi,theta,psi], [0,0,0])
subs(H_E2, [phi,theta,psi], [0,0,0])

% for i= 1:9
%     isequaln(H_E(i) , H_E2(i))
% end


