
function [cost]= optimization_fn_fg(obj, x, params)

% from a vector to cells
obj.from_vector_to_estimator(x, params)

% initialize
cost= 0;

% cost of the prior msmt
z= obj.z_fg(1:params.m);
z_expected= x(1:params.m);
cost= cost +  ((z - z_expected)' / obj.PX_prior) * (z - z_expected);

for i= params.M:-1:2
    
    % ------------ gyro cost ------------
    z= obj.z_gyro_ph{i};
    z_expected= ( obj.x_ph{i-1}(params.ind_yaw) - obj.x_ph{i}(params.ind_yaw) ) / params.dt_sim;
    cost= cost + ((z - z_expected)' / params.sig_gyro_z)* (z - z_expected);
    % ------------------------------------
    
    % ---------- odometry cost ----------
    z_expected= obj.return_odometry_update_sim(obj.x_ph{i}, obj.odometry_ph{i}, params) - obj.x_ph{i-1};
    [~, D_bar, ~, ~]= obj.return_Phi_and_D_bar(...
        obj.x_ph{i}, obj.odometry_ph{i}(1), obj.odometry_ph{i}(2), params);    
    cost= cost + z_expected' * pinv(D_bar) * z_expected;
    % ------------------------------------
    
    % ------------ lidar cost ------------
    z= obj.z_lidar_ph{i-1};
    n_L= length(z)/ params.m_F;
    z_expected= obj.return_expected_z_lidar(obj.x_ph{i-1}, obj.association_ph{i-1}, params);
    cost= cost + ((z - z_expected)' /  kron(eye(n_L) , params.R_lidar) ) * (z - z_expected);
    % ------------------------------------
    
end


% ------------ gyro cost ------------
z= obj.z_gyro;
z_expected= ( obj.XX(params.ind_yaw) - obj.x_ph{1}(params.ind_yaw) ) / params.dt_sim;
cost= cost + ((z - z_expected)' / params.sig_gyro_z)* (z - z_expected);
% ------------------------------------

% ---------- odometry cost ----------
z_expected= obj.return_odometry_update_sim(obj.x_ph{1}, obj.odometry_k, params) - obj.XX;
[~, D_bar, ~, ~]= obj.return_Phi_and_D_bar(...
    obj.x_ph{1}, obj.odometry_k(1), obj.odometry_k(2), params);
cost= cost + z_expected' * pinv(D_bar) * z_expected;
% ------------------------------------

% ------------ lidar cost ------------
z= obj.z_lidar';
z= z(:);
n_L= length(z)/ params.m_F;
z_expected= obj.return_expected_z_lidar(obj.XX, obj.association, params);
cost= cost + ((z - z_expected)' /  kron(eye(n_L) , params.R_lidar) ) * (z - z_expected);
% ------------------------------------

end

