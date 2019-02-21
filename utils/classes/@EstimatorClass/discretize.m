function discretize(obj, F, G, S, dT)
% MATRICES2DISCRETE This function discretize the continuous time model. It
% works for either the GPS or IMU discretization times.
% updates Phi_k & D_bar

% sysc= ss(F, zeros(15,1), zeros(1,15), 0);
% sysd= c2d(sysc, dT);
% Phi_k= sysd.A;

% Methdo to obtain covariance matrix for dicrete system
C= [-F, G*S*G';
    zeros(15), F'];

% Proper method
EXP= expm(C*dT);
obj.Phi_k= EXP(16:end,16:end)';
obj.D_bar= obj.Phi_k * EXP(1:15,16:end);

% Simplified method
%             obj.D_bar= (G*dT) * (S/dT) * (G*dT)'; % simplified version
end