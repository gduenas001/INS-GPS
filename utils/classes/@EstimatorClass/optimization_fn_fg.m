
function [cost]= optimization_fn_fg(obj, x, params)

% from a vector to cells
inds= 1:params.m;
for i= params.M:-1:1
    % update the cell
    obj.x_ph{i}= x(inds);
    
    % update index
    inds= inds + params.m;
end
% current pose
obj.XX= x(end - params.m + 1:end);



% initialize
cost= 0;

% cost of the prior msmt
z= obj.z_fg(1:params.m);
x_prior= x(1:params.m);
cost= cost +  ((z - x_prior)' / obj.PX_prior) * (z - x_prior);

for i= params.M:-1:2
    
    % ------------ gyro cost ------------
    z= obj.z_gyro_ph{i};
    z_expected= ( obj.x_ph{i-1}(params.ind_yaw) - obj.x_ph{i}(params.ind_yaw) ) / params.dt_sim;
    cost= cost + ((z - z_expected)' / params.sig_gyro_z)* (z - z_expected);
    
    if length(cost) ~= 1, error(''), end
    % ------------------------------------
    
    % ---------- odometry cost ----------
    z= zeros(3,1);
    z_expected=...
        obj.return_odometry_update_sim(obj.x_ph{i}, obj.odometry_ph{i}, params) - obj.x_ph{i-1};
    [~, D_bar, ~, ~]= obj.return_Phi_and_D_bar(...
        obj.x_ph{i}, obj.odometry_ph{i}(1), obj.odometry_ph{i}(2), params);    
    cost= cost + (z - z_expected)' * pinv(D_bar) * (z - z_expected);
    if length(cost) ~= 1, error(''), end
    % ------------------------------------
    
    % ------------ lidar cost ------------
    z= obj.z_lidar_ph{i-1};
    n_L= length(z)/ params.m_F;
    z_expected= obj.return_expected_z_lidar(obj.x_ph{i-1}, obj.association_ph{i-1}, params);
    cost= cost + ((z - z_expected)' /  kron(eye(n_L) , params.R_lidar) ) * (z - z_expected);
    if length(cost) ~= 1, error(''), end
    % ------------------------------------
    
end


% ------------ gyro cost ------------
z= obj.z_gyro_ph{params.M-1};
z_expected= ( obj.XX(params.ind_yaw) - obj.x_ph{params.M}(params.ind_yaw) ) / params.dt_sim;
cost= cost + ((z - z_expected)' / params.sig_gyro_z)* (z - z_expected);
if length(cost) ~= 1, error(''), end
% ------------------------------------

% ---------- odometry cost ----------
z= zeros(3,1);
z_expected=...
    obj.return_odometry_update_sim(obj.x_ph{params.M}, obj.odometry_ph{params.M}, params) - obj.XX;
[~, D_bar, ~, ~]= obj.return_Phi_and_D_bar(...
    obj.x_ph{params.M}, obj.odometry_ph{params.M}(1), obj.odometry_ph{params.M}(2), params);
cost= cost + (z - z_expected)' * pinv(D_bar) * (z - z_expected);
if length(cost) ~= 1, error(''), end
% ------------------------------------

% ------------ lidar cost ------------
z= obj.z_lidar(:);
n_L= length(z)/ params.m_F;
z_expected= obj.return_expected_z_lidar(obj.XX, obj.association, params);
cost= cost + ((z - z_expected)' /  kron(eye(n_L) , params.R_lidar) ) * (z - z_expected);
if length(cost) ~= 1, error(''), end
% ------------------------------------


% % returnt he Jacobian if required
% if nargout > 1    
%     A= obj.return_A_fg(x, params);
% end

end

