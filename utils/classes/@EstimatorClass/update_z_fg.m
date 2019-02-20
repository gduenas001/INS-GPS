
function update_z_fg(obj, counters, params)


if counters.k_lidar > params.preceding_horizon_size

    % initialize the z vector with the prior
    obj.z_fg= obj.x_ph{params.M};
    
    % put all the measurements together
    for i= params.M - 1:-1:1
        obj.z_fg= [obj.z_fg; zeros(3,1)]; % this includes gyro + inputs (3 ind msmts)
        obj.z_fg= [obj.z_fg; obj.z_lidar_ph{i}];
    end
    
    % add the current msmts
    obj.z_fg= [obj.z_fg; zeros(3,1)];
    obj.z_fg= [obj.z_fg; obj.z_lidar(:)];
end

end