
function [XX,PX]= lidarUpdate(XX,PX,z,association,appearances,R,SWITCH_CALIBRATION)
XX(9)= pi_to_pi(XX(9));

if all(association == -1), return; end

% Eliminate the non-associated features
z(association == -1,:)= [];
association(association == -1) = [];

lenz= length(association);
lenx= length(XX);

R= kron(R,eye(lenz));
H= zeros(2*lenz,lenx);
% H(:,1:2)= ones(2*lenz,2); % This artificially adds the uncertainty in x


for i= 1:length(association)    
    indz= 2*i + (-1:0);
    indx= 15 + 2*association(i) + (-1:0);
    H(indz,indx)= eye(2);
end

% Update
L= PX*H' / (H*PX*H' + kron(PX(1:2,1:2),eye(lenz)) + R);
zVector= z'; zVector= zVector(:);
innov= zVector - H*XX;

% If it is calibrating, update only landmarks
if SWITCH_CALIBRATION
    XX0= XX(1:15);
    PX0= PX(1:15,1:15);
    XX= XX + L*innov;
    PX= PX - L*H*PX;
    XX(1:15)= XX0;
    PX(1:15,1:15)= PX0;
else 
    XX= XX + L*innov;
    PX= PX - L*H*PX;
end












