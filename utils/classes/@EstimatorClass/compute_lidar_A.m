
function A_k= compute_lidar_A(obj, x, association, params)
% this function computes one part of the jacobian for the lidar msmts at
% the corresponding time where estimate x and association occur


% number of landmarks and msmts at this time
n_L= length(association);
n= n_L * params.m_F;

% needed parameters
spsi= sin(x( params.ind_yaw ));
cpsi= cos(x( params.ind_yaw ));

% initialize
A_k= inf * ones( n , params.m );
for i= 1:obj.n_L
    % Indexes
    inds= 2*i + (-1:0);
    
    dx= obj.landmark_map(association(i), 1) - x(1);
    dy= obj.landmark_map(association(i), 2) - x(2);
    
    % Jacobian -- H
    A_k(inds,1)= [-cpsi; spsi];
    A_k(inds,2)= [-spsi; -cpsi];
    A_k(inds,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                               -dx * cpsi - dy * spsi];
                               
    % whiten Jacobian
    A_k(inds, :)= params.sqrt_inv_R_lidar * A_k(inds, :);
end

end


