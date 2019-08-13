
function A= return_lidar_A(obj, x, association, params, ind_of_faulted_LM)
% this function computes one part of the jacobian for the lidar msmts at
% the corresponding time where estimate x and association occur


% number of landmarks and msmts at this time
n_L= length(association);
n= n_L * params.m_F;

% needed parameters
spsi= sin(x( params.ind_yaw ));
cpsi= cos(x( params.ind_yaw ));

% initialize
if ind_of_faulted_LM==0
    A= inf * ones( n , params.m );
else
    A= inf * ones( n-params.m_F , params.m );
end

j=1;
for i = 1:n_L
    
    if i == ind_of_faulted_LM
        continue;
    end
    
    % Indexes
    inds= 2*j + (-1:0);
    
    dx= obj.landmark_map(association(i), 1) - x(1);
    dy= obj.landmark_map(association(i), 2) - x(2);
    
    % Jacobian -- H
    A(inds,1)= [-cpsi; spsi];
    A(inds,2)= [-spsi; -cpsi];
    A(inds,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                               -dx * cpsi - dy * spsi];
                               
    % whiten Jacobian
    A(inds, :)= params.sqrt_inv_R_lidar * A(inds, :);
    
    j= j+1;
    
end

end


