function [Phi, D_bar]= discretize( F, G, H, S, dT)
%MATRICES2DISCRETE This function discretize the continuous time model. It
%works for either the GPS or IMU discretization times.


sysc= ss(F, zeros(15,1), H, 0);
sysd= c2d(sysc, dT);
Phi= sysd.A;

% Methdo to obtain covariance matrix for dicrete system
C= [-F, G*S*G';
     zeros(15), F'];

% Proper method
% EXP= expm(C*dT);
% D_bar= EXP(16:end,16:end)' * EXP(1:15,16:end);

% Simplified method
D_bar= (G*dT) * (S/dT) * (G*dT)'; % simplified version


end

