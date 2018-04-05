function [Fn, F, Gu_tilda, Gw, H]= Matrices(x, ax, ay, az, wx, wy, wz, tau)
%MATRICES Summary of this function goes here
% Detailed explanation goes here

% Linerize around this Euler angles 
phi= x(7); theta= x(8); psi= x(9);

% Rotation matrix from body-frame to nav-frame
R_NB= R_NB_rot(phi,theta,psi);
% From body rates to Euler rates
invQ_BE= invQ_BE_fn(phi,theta);
% Skew-symmetric matrix for specific force
fx_B= (-1)* [0,  -az, ay; 
             az, 0,  -ax;
            -ay, ax,  0];
% K* or F_E
K_star= [(sin(theta)*(wy*cos(phi) - wz*sin(phi)))/cos(theta),  (wz*cos(phi) + wy*sin(phi))/cos(theta)^2, 0;
         -wz*cos(phi) - wy*sin(phi), 0, 0;
         (wy*cos(phi) - wz*sin(phi))/cos(theta), (sin(theta)*(wz*cos(phi) + wy*sin(phi)))/cos(theta)^2, 0];
 
% Upper left Fn (9x9)
Fn= zeros(9);
Fn(1:3,4:6)= eye(3);
Fn(4:6,7:9)= R_NB*fx_B;
Fn(7:9,7:9)= K_star;

% Control
Gu= zeros(9,6);
Gu(4:6,1:3)= R_NB;
Gu(7:9,4:6)= invQ_BE;
Gu_tilda= [Gu; zeros(6)];

% Complete F (15x15)
Fb= -eye(6)/tau;
F= [Fn,        -Gu;
    zeros(6,9), Fb];

% Noise
Gw= [-Gu, zeros(9,6);
      zeros(6,6), eye(6)];

% Measurement matrix
H= zeros(3,15);
H(1:3,1:3)= eye(3);

end

