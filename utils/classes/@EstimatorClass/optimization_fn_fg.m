
function [cost, A]= optimization_fn_fg(obj, x, params)

% from a vector to cells
obj.from_vector_to_estimator(x, params)

% initialize
cost= 0;

% cost of the prior msmt
z= obj.x_prior;
z_expected= x(1:params.m);
cost= cost +  (z - z_expected)' * obj.Gamma_prior * (z - z_expected);
if nargout > 1
    % initialize normalized Jacobian
    A= zeros( obj.n_total, obj.m_M );
    
    % plug the prior in A
    A( 1:params.m, 1:params.m )= sqrtm( obj.Gamma_prior );
    
    % pointers to the next part of A to be filled
    r_ind= params.m + 1;
    c_ind= 1;
end


for i= params.M:-1:2
    
    % ------------ gyro cost ------------
    z= obj.z_gyro_ph{i};
    z_expected= ( obj.x_ph{i-1}(params.ind_yaw) - obj.x_ph{i}(params.ind_yaw) ) / params.dt_sim;
    cost= cost + (z - z_expected)^2 / params.sig_gyro_z^2;
    % ------------------------------------

    % ---------- gyro submatrix ----------
    if nargout > 1
        A( r_ind, c_ind : c_ind + params.m - 1 )= ...
            -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
        
        A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
            params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
        
        % update the row index to point towards the next msmt
        r_ind= r_ind + 1;
    end
    % ------------------------------------

    
    
    % ---------- odometry cost ----------
    z_expected= obj.return_odometry_update_sim(obj.x_ph{i}, obj.odometry_ph{i}, params) - obj.x_ph{i-1};
    [~, D_bar]= obj.return_Phi_and_D_bar(...
        obj.x_ph{i}, obj.odometry_ph{i}(1), obj.odometry_ph{i}(2), params);
    cost= cost + z_expected' * pinv(D_bar) * z_expected;
    % ------------------------------------
    
    % ---------- odometry submatrix ----------
    if nargout > 1
        [Phi, D_bar]= obj.return_Phi_and_D_bar(...
            obj.x_ph{i}, obj.odometry_ph{i}(1), obj.odometry_ph{i}(2), params);
        
        [~,S,V]= svd( D_bar );
        r_S= rank(S);
        sqrt_inv_D_bar= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
        
        A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1 )= ...
            sqrt_inv_D_bar * Phi;
        
        A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m -1)= ...
            -sqrt_inv_D_bar;
        
        % update the row & column indexes
        r_ind= r_ind + r_S;
        c_ind= c_ind + params.m;
    end
    % ------------------------------------

    
    
    % ------------ lidar cost ------------
    z= obj.z_lidar_ph{i-1};
    n_L= length(z)/ params.m_F;
    z_expected= obj.return_expected_z_lidar(obj.x_ph{i-1}, obj.association_ph{i-1}, params);
    cost= cost + (z - z_expected)' *  kron( eye(n_L) , inv(params.R_lidar) ) * (z - z_expected);
    % ------------------------------------
    
    % ---------- lidar submatrix ----------
    if nargout > 1
        A_lidar= obj.return_lidar_A(obj.x_ph{i-1}, obj.association_ph{i-1}, params);
        n_L= length(obj.association_ph{i-1});
        n= n_L * params.m_F;
        A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;
        
        % update the row index
        r_ind= r_ind + n;
    end
    % ------------------------------------
end


% ------------ gyro cost ------------
z= obj.z_gyro;
z_expected= ( obj.XX(params.ind_yaw) - obj.x_ph{1}(params.ind_yaw) ) / params.dt_sim;
cost= cost + (z - z_expected)^2 / params.sig_gyro_z^2;
% ------------------------------------

% ---------- gyro submatrix ----------
if nargout > 1
    A( r_ind, c_ind : c_ind + params.m - 1 )= ...
        -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
        params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    % update the row index to point towards the next msmt
    r_ind= r_ind + 1;
end
% ------------------------------------



% ---------- odometry cost ----------
z_expected= obj.return_odometry_update_sim(obj.x_ph{1}, obj.odometry_k, params) - obj.XX;
[~, D_bar]= obj.return_Phi_and_D_bar(...
    obj.x_ph{1}, obj.odometry_k(1), obj.odometry_k(2), params);
cost= cost + z_expected' * pinv(D_bar) * z_expected;
% ------------------------------------

% ---------- odometry submatrix ----------
if nargout > 1
    [Phi, D_bar]= obj.return_Phi_and_D_bar(...
        obj.x_ph{1}, obj.odometry_k(1), obj.odometry_k(2), params);
    
    [~,S,V]= svd( D_bar );
    r_S= rank(S);
    sqrt_inv_D_bar= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
    
    A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1 )= ...
        sqrt_inv_D_bar * Phi;
    
    A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m -1)= ...
        -sqrt_inv_D_bar;
    
    % update the row & column indexes
    r_ind= r_ind + r_S;
    c_ind= c_ind + params.m;
end
% ------------------------------------



% ------------ lidar cost ------------
z= obj.z_lidar';
z= z(:);
n_L= length(z)/ params.m_F;
z_expected= obj.return_expected_z_lidar(obj.XX, obj.association, params);
cost= cost + (z - z_expected)' *  kron( eye(n_L) , inv(params.R_lidar) ) * (z - z_expected);
% ------------------------------------

% ---------- lidar submatrix ----------
if nargout > 1
    A_lidar= obj.return_lidar_A(obj.XX, obj.association, params);
    n_L= length(obj.association);
    n= n_L * params.m_F;
    A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;
end
% ------------------------------------


% divide cost by 2 to make Jacobian consistent
cost= cost / 2;

end

