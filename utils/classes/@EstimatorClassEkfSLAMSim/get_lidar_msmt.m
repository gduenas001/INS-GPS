
function z_lidar= get_lidar_msmt(obj, params)


spsi= sin(obj.x_true(3));
cpsi= cos(obj.x_true(3));
z_lidar= [];
%obj.association_true= [];
obj.num_faults_k= 0;
for l= 1:obj.num_landmarks
    % check if the landmark is in the FoV
    dx= obj.landmark_map(l,1) - obj.x_true(1);
    if abs(dx) > params.lidarRange, continue, end
    dy= obj.landmark_map(l,2) - obj.x_true(2);
    if abs(dy) > params.lidarRange, continue, end
    if sqrt( dx^2 + dy^2 ) > params.lidarRange, continue, end        

    % simulate msmt with noise
    z_lm(1)=  dx*cpsi + dy*spsi + normrnd(0, params.sig_lidar);
    z_lm(2)= -dx*spsi + dy*cpsi + normrnd(0, params.sig_lidar);
        
    % add possible fault
    if params.SWITCH_LIDAR_FAULTS
        if binornd(1, params.P_UA)
            z_lm= z_lm + rand() * params.sig_lidar * 10;
            obj.num_faults_k= obj.num_faults_k + 1;
        end
    end
    
    % add measurement
    z_lidar= [z_lidar; z_lm];

    % save the true association
    % obj.association_true= [obj.association_true; l];
end

% add them to the estimator class property
obj.z_lidar= z_lidar;

% if we use the NN associaiton this values get overwritten 
% obj.n_L_k= length(obj.association_true);
obj.n_L_k= size(obj.z_lidar, 1);
%obj.n_k= obj.n_L_k * params.m_F;
obj.n_k= obj.n_L_k * params.m_F;
end