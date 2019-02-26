
function compute_lidar_H_k_offline_exp(obj, params, FG, epoch) % TODO: osama - change this name, this computes H not A
% this funcion builds the Jacobian H for the factor graphs case without
% mesaurements. It uses all the landmarks in the field of view of the lidar

spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));

% landmarks in the field of view
obj.lm_ind_fov= FG.associations{epoch};

% number of expected extracted landmarks
obj.n_L_k= length(FG.associations{epoch});

% number of expected measurements
obj.n_k= obj.n_L_k * params.m_F;

% build Jacobian
obj.H_k_lidar= zeros( obj.n_k , params.m );
for i= 1:obj.n_L_k
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= obj.landmark_map(obj.lm_ind_fov(i), 1) - obj.XX(1);
    dy= obj.landmark_map(obj.lm_ind_fov(i), 2) - obj.XX(2);
    
    % Jacobian -- H
    obj.H_k_lidar(indz,1)= [-cpsi; spsi];
    obj.H_k_lidar(indz,2)= [-spsi; -cpsi];
    obj.H_k_lidar(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
                               
end

% compute the whiten jacobian
obj.H_k_lidar= kron( eye( obj.n_L_k ) , params.sqrt_inv_R_lidar ) * obj.H_k_lidar;

end