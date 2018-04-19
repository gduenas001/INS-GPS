
clear; clc; close all;

% State vector
syms('rx','ry','rz','real')
syms('vx','vy','vz','real')
syms('phi', 'theta', 'psi','real')
syms('bx_f','by_f','bz_f','bx_w','by_w','bz_w','real')

% IMU readings & g
syms('ax', 'ay', 'az', 'wx', 'wy', 'wz', 'g', 'real')

% Noise
syms('vx_f','vy_f','vz_f','vx_w','vy_w','vz_w','real') % sensor noise
syms('nx_f','ny_f','nz_f','nx_w','ny_w','nz_w','real') % bias noise

% Bias Gauss process
syms('tau_f', 'tau_w','real')

% syms('taux_f','tauy_f','tauz_f','taux_w','tauy_w','tauz_w','real')
% syms('ax0', 'ay0', 'az0', 'wx0', 'wy0', 'wz0', 'real')
% syms('phi0', 'theta0', 'psi0','real')

% Build vectors
f= [ax; ay; az]; % Accelerations measured by IMU
w= [wx; wy; wz]; % Body rates measure by IMU
r= [rx;ry;rz];
v= [vx;vy;vz];
E= [phi; theta; psi]; % Euler angles
b_f= [bx_f;by_f;bz_f];
b_w= [bx_w;by_w;bz_w];
v_f= [vx_f;vy_f;vz_f];
v_w= [vx_w;vy_w;vz_w];
n_f= [nx_f;ny_f;nz_f];
n_w= [nx_w;ny_w;nz_w];
% tau_f= [taux_f;tauy_f;tauz_f];
% tau_w= [taux_w;tauy_w;tauz_w];
g_N= [0; 0; g]; % where g is positive


% Rotation matrices for each axis
R_psi= [cos(psi), sin(psi), 0;
        -sin(psi), cos(psi), 0;
        0, 0, 1];
R_theta= [cos(theta), 0, -sin(theta);
          0, 1, 0;
          sin(theta), 0, cos(theta)];
R_phi= [1, 0, 0;
        0, cos(phi), sin(phi);
        0, -sin(phi), cos(phi)];

% Rotation matrices B to N and N to B
R_NB= simplify( R_psi*R_theta*R_phi );
R_BN= R_NB';

% From body rates to Euler rates
Q_EB= simplify( inv( [1, 0, sin(theta);
                0, cos(phi), -sin(phi)*cos(theta);
                0, sin(phi), cos(phi)*cos(theta)] ) );

% Q_EB= [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
%           0, cos(phi), -sin(phi);
%           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];



%% Continuous nonlinear model
r_dot= v;
v_dot= simplify( R_NB*( f - b_f - v_f ) + g_N );
E_dot= simplify( Q_EB * (w - b_w - v_w) );
b_f_dot= -tau_f * b_f + n_f;
b_w_dot= -tau_w * b_w + n_w;

% All together
x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];

%% Linearize continuous time model
F= simplify( [diff(x_dot, rx), diff(x_dot, ry), diff(x_dot, rz)...
              diff(x_dot, vx) , diff(x_dot, vy), diff(x_dot, vz)...
              diff(x_dot, phi) , diff(x_dot, theta), diff(x_dot, psi)...
              diff(x_dot, bx_f) , diff(x_dot, by_f), diff(x_dot, bz_f)...
              diff(x_dot, bx_w) , diff(x_dot, by_w), diff(x_dot, bz_w) ] );

Gv= simplify( [diff(x_dot, vx_f), diff(x_dot, vy_f), diff(x_dot, vz_f)...
               diff(x_dot, vx_w) , diff(x_dot, vy_w), diff(x_dot, vz_w) ] );
   
Gn= simplify( [diff(x_dot, nx_f), diff(x_dot, ny_f), diff(x_dot, nz_f)...
               diff(x_dot, nx_w) , diff(x_dot, ny_w), diff(x_dot, nz_w) ] );
    
G= [Gv, Gn];

% Substitute for zero noise
F= subs(F, [vx_f, vy_f, vy_w, vz_f, vz_w], zeros(1,5));


%% Virtual measurement update Jacobians
h= simplify( [ 0, 0, 1 ] * R_BN * v );
H= simplify( [diff(h, rx),   diff(h, ry),    diff(h, rz)...
              diff(h, vx),   diff(h, vy),    diff(h, vz)...
              diff(h, phi),  diff(h, theta), diff(h, psi)...
              diff(h, bx_f), diff(h, by_f),  diff(h, bz_f)...
              diff(h, bx_w), diff(h, by_w),  diff(h, bz_w) ] );


%% This generates fucntion files automatically
% matlabFunction(R_NB, 'file', 'R_NB_fn')
matlabFunction(Q_EB, 'file', 'Q_EB_fn')
% matlabFunction(F,    'file', 'F_fn')
% matlabFunction(G,    'file', 'G_fn')
% matlabFunction(H,    'file', 'H_fn')






