function lidar_update_localization(obj, z, association, params)

R= params.R_lidar; % TODO: move to the kron

obj.XX(9)= pi_to_pi( obj.XX(9) );

if all(association == 0), return; end

% Eliminate the non-associated features
z(association == 0, :)= [];
association(association == 0) = [];

obj.n_k= length(association) * params.m_F;
lenx= length(obj.XX);

R= kron( R, eye( obj.n_k / params.m_F ) );
obj.H_k= zeros(obj.n_k, lenx);

%Build Jacobian H
spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));
zHat= zeros(obj.n_k,1);
for i= 1:length(association)
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= obj.landmark_map(association(i), 1) - obj.XX(1);
    dy= obj.landmark_map(association(i), 2) - obj.XX(2);
    
    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
        -dx*spsi + dy*cpsi];
    
    % Jacobian -- H
    obj.H_k(indz,1)= [-cpsi; spsi];
    obj.H_k(indz,2)= [-spsi; -cpsi];
    obj.H_k(indz,9)= [-dx * spsi + dy * cpsi;
        -dx * cpsi - dy * spsi];
end

% Update
obj.Y_k= obj.H_k * obj.PX * obj.H_k' + R;
obj.L_k= obj.PX * obj.H_k' / obj.Y_k;
zVector= z'; zVector= zVector(:);
obj.gamma_k= zVector - zHat;
obj.q_k= obj.gamma_k' / obj.Y_k * obj.gamma_k;
obj.XX= obj.XX + obj.L_k * obj.gamma_k;
obj.PX= obj.PX - obj.L_k * obj.H_k * obj.PX;
end