
function update_preceding_horizon(obj, params)

% update odometry msmts in the ph
obj.odometry_ph= {inf, obj.odometry_k, obj.odometry_ph{2:obj.M}};

% update odometry msmts in the ph
obj.z_gyro_ph= {inf, obj.z_gyro, obj.z_gyro_ph{2:obj.M}};

% update lidar msmts in the ph
z_lidar= obj.z_lidar';
obj.z_lidar_ph= {z_lidar(:), obj.z_lidar_ph{1:obj.M}};

% update the previous poses
obj.x_ph= {obj.XX, obj.x_ph{1:obj.M}};

% update the associations in the ph
obj.association_ph= {obj.association, obj.association_ph{1:obj.M}};

% update the true associations in the ph
obj.association_true_ph= {obj.association_true, obj.association_true_ph{1:obj.M}};

% update the number of associations in the ph
obj.n_L_k_ph= [ obj.n_L_k; obj.n_L_k_ph(1:obj.M - 1) ];

end





