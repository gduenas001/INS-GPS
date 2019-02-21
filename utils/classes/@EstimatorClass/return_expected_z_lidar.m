
function z_expected= return_expected_z_lidar(obj, x, association, params)

spsi= sin(x(params.ind_yaw));
cpsi= cos(x(params.ind_yaw));
z_expected= [];
for i= 1:length(association)
    % get index of the landmark
    l= association(i);
    
    % compute expected msmt
    dx= obj.landmark_map(l,1) - x(1);
    dy= obj.landmark_map(l,2) - x(2);
    
    z_lm=  [dx*cpsi + dy*spsi; -dx*spsi + dy*cpsi];
    
    % add measurement
    z_expected= [z_expected; z_lm];    
end

end








