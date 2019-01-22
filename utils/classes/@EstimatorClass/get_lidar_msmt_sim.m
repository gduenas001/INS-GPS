
function z_lidar= get_lidar_msmt_sim(obj, params)


spsi= sin(obj.x_true(3));
cpsi= cos(obj.x_true(3));
z_lidar= [];

for l= 1:obj.num_landmarks
    % check if the landmark is in the FoV
    dx= obj.landmark_map(l,1) - obj.x_true(1);
    if abs(dx) > params.lidarRange, continue, end
    dy= obj.landmark_map(l,2) - obj.x_true(2);
    if abs(dy) > params.lidarRange, continue, end
    
    % simulate msmt with noise
    z_lm(1)=  dx*cpsi + dy*spsi + randn(1) * params.sig_lidar;
    z_lm(2)= -dx*spsi + dy*cpsi + randn(1) * params.sig_lidar;
    
    % add measurement
    z_lidar= [z_lidar; z_lm];
end

end