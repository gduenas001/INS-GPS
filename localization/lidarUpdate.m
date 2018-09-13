
function lidarUpdate(z,idf,appearances,R,SWITCH_CALIBRATION)

global XX PX LM

XX(9)= pi_to_pi(XX(9));


if all(idf == -1), return; end

% Eliminate the non-associated features
z(idf == -1 | idf == 0,:)= []; 
idf(idf == -1 | idf == 0) = []; 

lenz= length(idf);
lenx= length(XX);

R= kron(R,eye(lenz));
H= zeros(2*lenz,lenx);


% Build Jacobian H
spsi= sin(XX(9));
cpsi= cos(XX(9));
zHat= zeros(2*lenz,1);
for i= 1:length(idf)
    % Indexes 
    indz= 2*i + (-1:0);
%     indx= 15 + 2*idf(i) + (-1:0);
    
    dx= LM(idf(i),1) - XX(1);
    dy= LM(idf(i),2) - XX(2);
        
    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
                 -dx*spsi + dy*cpsi];
    
    % Jacobian -- H
    H(indz,1)= [-cpsi; spsi];
    H(indz,2)= [-spsi; -cpsi];
    H(indz,9)= [-dx * spsi + dy * cpsi;
                -dx * cpsi - dy * spsi];
%     H(indz,indx)= [cpsi, spsi;
%                   -spsi, cpsi];
    
end

% Update
Y= H*PX*H' + R;
L= PX * H' / Y;
zVector= z'; zVector= zVector(:);
innov= zVector - zHat;

% If it is calibrating, update only landmarks
if SWITCH_CALIBRATION
%     XX0= XX(1:15);
%     PX0= PX(1:15,1:15);
%     XX= XX + L*innov;
%     PX= PX - L*H*PX;
%     XX(1:15)= XX0;
%     PX(1:15,1:15)= PX0;
else 
    XX= XX + L*innov;
    PX= PX - L*H*PX;
end












