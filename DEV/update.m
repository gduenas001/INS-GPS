
function update(z,R,idf, batch)
% function update(z,R,idf, batch)
%
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   XX, PX - updated state and covariance (global variables)

if batch == 1
    batch_update(z,R,idf);
else
    single_update(z,R,idf);
end
end
%
%



function batch_update(z,R,idf)
global XX PX

lenz= size(z,2);
lenx= length(XX);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(XX, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
KF_cholesky_update(v,RR,H);
end




%

function single_update(z,R,idf)
global XX PX

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(XX, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    KF_cholesky_update(v,RR,H);
end        
end




function [z,H]= observe_model(x, idf)
%function [z,H]= observe_model(x, idf)
%
% INPUTS:
%   x - state vector
%   idf - index of feature order in state
%
% OUTPUTS:
%   z - predicted observation
%   H - observation Jacobian
%
% Given a feature index (ie, the order of the feature in the state vector),
% predict the expected range-bearing observation of this feature and its Jacobian.

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state
H= zeros(2, length(x));

% auxiliary values
dx= x(fpos)  -x(1); 
dy= x(fpos+1)-x(2);
d2= dx^2 + dy^2;
d= sqrt(d2);
xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
z= [d;
    atan2(dy,dx) - x(3)];

% calculate H
H(:,1:3)        = [-xd -yd 0; yd2 -xd2 -1];
H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];
end





function KF_cholesky_update(v,R,H)
%function KF_cholesky_update(v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P]
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using Cholesky factorisation, which
% is more numerically stable than a naive implementation.
global XX PX

%PHt= PX*H';
PHt= (H*PX)'; % Matlab is column-major, so (H*PX)' is more efficient than PX*H' [Tim 2004]
S= H*PHt + R;

S= (S+S')*0.5; % ensure is symmetric
SChol= chol(S);

SCholInv= inv(SChol); % triangular matrix
W1= PHt * SCholInv;
W= W1 * SCholInv';

XX= XX + W*v; % update 
PX= PX - W1*W1';
end
