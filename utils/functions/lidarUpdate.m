
function lidarUpdate(z,association,appearances,R,SWITCH_CALIBRATION)

global XX PX

XX(9)= pi_to_pi(XX(9));


if all(association == -1), return; end

% Eliminate the non-associated features
z(association == -1 | association == 0,:)= []; 
association(association == -1 | association == 0) = []; 

lenz= length(association);
lenx= length(XX);

R= kron(R,eye(lenz));
H= zeros(2*lenz,lenx);

%Build Jacobian H
spsi= sin(XX(9));
cpsi= cos(XX(9));
zHat= zeros(2*lenz,1);
for i= 1:length(association)
    % Indexes 
    indz= 2*i + (-1:0);
    indx= 15 + 2*association(i) + (-1:0);
    
    dx= XX(indx(1)) - XX(1);
    dy= XX(indx(2)) - XX(2);
    
    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
                 -dx*spsi + dy*cpsi];
    
    % Jacobian -- H
    H(indz,1)= [-cpsi; spsi];
    H(indz,2)= [-spsi; -cpsi];
    H(indz,9)= [-dx * spsi + dy * cpsi;
                -dx * cpsi - dy * spsi];
    H(indz,indx)= [cpsi, spsi;
                  -spsi, cpsi];
    
end

% Update
Y= H*PX*H' + R;
L= PX * H' / Y;
zVector= z'; zVector= zVector(:);
innov= zVector - zHat;

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












