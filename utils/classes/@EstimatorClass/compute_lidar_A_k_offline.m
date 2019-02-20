
function compute_lidar_A_k_offline(obj, params)
% this funcion builds the Jacobian H for the factor graphs case without
% mesaurements. It uses all the landmarks in the field of view of the lidar

spsi= sin(obj.x_true(3));
cpsi= cos(obj.x_true(3));

% find the landmarks in the field of view
obj.lm_ind_fov= [];
for l= 1:obj.num_landmarks
    % check if the landmark is in the FoV
    dx= obj.landmark_map(l,1) - obj.x_true(1);
    if abs(dx) > params.lidarRange, continue, end
    dy= obj.landmark_map(l,2) - obj.x_true(2);
    if abs(dy) > params.lidarRange, continue, end
    if sqrt( dx^2 + dy^2 ) > params.lidarRange, continue, end        
    
    % add the landmark to the fov
    obj.lm_ind_fov= [obj.lm_ind_fov; l];
end

% number of expected extracted landmarks
obj.n_L_k= length(obj.lm_ind_fov);

% number of expected measurements
obj.n_k= obj.n_L_k * params.m_F;

% build Jacobian
obj.H_k= inf * ones( obj.n_k , params.m );
for i= 1:obj.n_L_k
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= obj.landmark_map(obj.lm_ind_fov(i), 1) - obj.x_true(1);
    dy= obj.landmark_map(obj.lm_ind_fov(i), 2) - obj.x_true(2);
    
    % Jacobian -- H
    obj.H_k(indz,1)= [-cpsi; spsi];
    obj.H_k(indz,2)= [-spsi; -cpsi];
    obj.H_k(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
                               
end