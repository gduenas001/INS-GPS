
function update_fg(obj, counters, params)
% updates the msmts and the Jacobian for the newly received msmts


% current lidar msmts normalized
b_k_lidar= kron( eye( obj.n_L_k ) , params.sqrt_inv_R_lidar ) * obj.z_lidar(:);

if counters.k_lidar > params.preceding_horizon_size

    % initialize the z vector with the prior
    obj.b_fg= sqrtm( inv(obj.PX_prior) ) * obj.x_ph{params.preceding_horizon_size};
    
    % put all the measurements together
    for i= params.preceding_horizon_size - 1:-1:1
        obj.b_fg= [obj.b_fg; zeros(3,1)];
        obj.b_fg= [obj.b_fg; obj.b_ph{i}];
    end
    
    % add the current msmts
    obj.b_fg= [obj.b_fg; zeros(3,1)];
    obj.b_fg= [obj.b_fg; b_k_lidar];

end

% total number of measurements
obj.n_total= length(obj.b_fg);

% total number of states to estimate
obj.m_M= (params.preceding_horizon_size + 1) * params.m;

% update the cells with the msmts
obj.b_ph= {b_k_lidar, obj.b_ph{1:params.preceding_horizon_size}};

% update the previous poses
obj.x_ph= {obj.XX, obj.x_ph{1:params.preceding_horizon_size}};

end