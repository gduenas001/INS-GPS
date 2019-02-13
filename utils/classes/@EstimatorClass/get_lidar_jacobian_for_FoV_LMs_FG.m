
function get_lidar_jacobian_for_FoV_LMs_FG(obj, params)

spsi= sin(obj.XX(3));
cpsi= cos(obj.XX(3));
obj.indicator_of_FoV_LMs= zeros(obj.num_landmarks,1);
% obj.association_true= [];

for l= 1:obj.num_landmarks
    % check if the landmark is in the FoV
    dx= obj.landmark_map(l,1) - obj.XX(1);
    if abs(dx) > params.lidarRange, continue, end
    dy= obj.landmark_map(l,2) - obj.XX(2);
    if abs(dy) > params.lidarRange, continue, end
    if sqrt( dx^2 + dy^2 ) > params.lidarRange, continue, end        
    
    obj.indicator_of_FoV_LMs(l)= 1;
                      
    % simulate msmt with noise
    %z_lm(1)=  dx*cpsi + dy*spsi + randn(1) * params.sig_lidar;
    %z_lm(2)= -dx*spsi + dy*cpsi + randn(1) * params.sig_lidar;
    
    % add measurement
    %z_lidar= [z_lidar; z_lm];
    
    % save the true association
    % obj.association_true= [obj.association_true; l];
end

obj.index_of_FoV_LMs= find(obj.indicator_of_FoV_LMs);
obj.n_k= length(obj.index_of_FoV_LMs)*params.m_F;
obj.H_k= inf * ones( length(obj.index_of_FoV_LMs)*params.m_F ,length(obj.XX) );

for i=1:length(obj.index_of_FoV_LMs)
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= obj.landmark_map(obj.index_of_FoV_LMs(i), 1) - obj.XX(1);
    dy= obj.landmark_map(obj.index_of_FoV_LMs(i), 2) - obj.XX(2);
    
    % Jacobian -- H
    obj.H_k(indz,1)= [-cpsi; spsi];
    obj.H_k(indz,2)= [-spsi; -cpsi];
    obj.H_k(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
                               
end