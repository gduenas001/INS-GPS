
function update_preceding_horizon(obj, params)

% update odometry msmts in the ph
obj.odometry_ph= {inf, obj.odometry_k, obj.odometry_ph{2:params.M}};

% update odometry msmts in the ph
obj.z_gyro_ph= {inf, obj.z_gyro, obj.z_gyro_ph{2:params.M}};

% update lidar msmts in the ph
z_lidar= obj.z_lidar';
obj.z_lidar_ph= {z_lidar(:), obj.z_lidar_ph{1:params.M}};

% update the previous poses
obj.x_ph= {obj.XX, obj.x_ph{1:params.M}};

% update the associations in the ph
obj.association_ph= {obj.association, obj.association_ph{1:params.M}};


end





