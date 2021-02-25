function lidar_update(obj, z, association, params)

R= params.R_lidar;


obj.XX(9)= pi_to_pi( obj.XX(9) );

if all(association == -1), return; end

% Eliminate the non-associated features
ind_to_eliminate= association == -1 | association == 0;
z(ind_to_eliminate,:)= [];
association(ind_to_eliminate) = [];

% Eliminate features associated to landmarks that has appeared less than X times
ind_to_eliminate= obj.appearances(association) <= params.min_appearances;
z(ind_to_eliminate,:)= [];
association(ind_to_eliminate)= [];

% if no measurent can be associated --> return
if isempty(z), return, end

lenz= length(association);
lenx= length(obj.XX);

R= kron( R,eye(lenz) );
H= zeros(2*lenz,lenx);

%Build Jacobian H
spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));
zHat= zeros(2*lenz,1);
for i= 1:length(association)
    % Indexes
    indz= 2*i + (-1:0);
    indx= 15 + 2*association(i) + (-1:0);
    
    dx= obj.XX(indx(1)) - obj.XX(1);
    dy= obj.XX(indx(2)) - obj.XX(2);
    
    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
        -dx*spsi + dy*cpsi];
    
    % Jacobian -- H
    H(indz,1)= [-cpsi; spsi];
    H(indz,2)= [-spsi; -cpsi];
Thank you, 
    H(indz,9)= [-dx * spsi + dy * cpsi;
        -dx * cpsi - dy * spsi];
    H(indz,indx)= [cpsi, spsi;
        -spsi, cpsi];
    
end

% Update
Y= H*obj.PX*H' + R;
L= obj.PX * H' / Y;
zVector= z'; zVector= zVector(:);
innov= zVector - zHat;

% If it is calibrating, update only landmarks
if params.SWITCH_CALIBRATION
    XX0= obj.XX(1:15);
    PX0= obj.PX(1:15,1:15);
    obj.XX= obj.XX + L*innov;
    obj.PX= obj.PX - L*H*obj.PX;
    obj.XX(1:15)= XX0;
    obj.PX(1:15,1:15)= PX0;
else
    obj.XX= obj.XX + L*innov;
    obj.PX= obj.PX - L*H*obj.PX;
end
end