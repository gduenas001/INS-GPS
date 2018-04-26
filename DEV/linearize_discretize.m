
function [Phi,D_bar]= linearize_discretize(x,u,S,taua,tauw,dT)

% Compute the F and G matrices (linear continuous time)
[F,G]= FG_fn(u(1),u(2),u(3),u(5),u(6),...
    x(7),x(8),x(9),x(10),x(11),x(12),x(14),x(15),taua,tauw);

% Discretize system for IMU time (only for variance calculations)
[Phi,D_bar]= discretize(F, G, S, dT);
end

function [Phi, D_bar]= discretize( F, G, S, dT)
%MATRICES2DISCRETE This function discretize the continuous time model. It
%works for either the GPS or IMU discretization times.


% sysc= ss(F, zeros(15,1), H, 0);
% sysd= c2d(sysc, dT);
% Phi= sysd.A;

% Methdo to obtain covariance matrix for dicrete system
C= [-F, G*S*G';
     zeros(15), F'];

% Proper method
EXP= expm(C*dT);
Phi= EXP(16:end,16:end)';
D_bar= Phi * EXP(1:15,16:end);


% Simplified method
% D_bar= (G*dT) * (S/dT) * (G*dT)'; % simplified version
end










