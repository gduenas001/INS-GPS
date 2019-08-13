
function A= return_A(obj, x, params)


% from a vector to cells
obj.from_vector_to_estimator(x, params)

% initialize normalized Jacobian
A= zeros( obj.n_total, obj.m_M );

% plug the prior in A
A( 1:params.m, 1:params.m )= sqrtm( obj.Gamma_prior );

% pointers to the next part of A to be filled
r_ind= params.m + 1;
c_ind= 1;

% build A whithen Jacobian
for i= obj.M : -1 : 2
    
    % ---------- gyro submatrix ----------
    A( r_ind, c_ind : c_ind + params.m - 1 )= ...
        -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
        params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    % update the row index to point towards the next msmt
    r_ind= r_ind + 1;
    % ------------------------------------
    
  
    
    % ---------- odometry submatrix ----------
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
    % ------------------------------------
    
    
    
    % ---------- lidar submatrix ----------
    A_lidar= obj.return_lidar_A(obj.x_ph{i-1}, obj.association_ph{i-1}, params);
    n_L= length(obj.association_ph{i-1});
    n= n_L * params.m_F;
    A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;
    
    % update the row index
    r_ind= r_ind + n;
    % ------------------------------------
end

% ---------- gyro submatrix ----------
A( r_ind, c_ind : c_ind + params.m - 1 )= ...
    -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];

A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
    params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];

% update the row index to point towards the next msmt
r_ind= r_ind + 1;
% ------------------------------------



% ---------- odometry submatrix ----------
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
% ------------------------------------


% ---------- lidar submatrix ----------
A_lidar= obj.return_lidar_A(obj.XX, obj.association, params);
n_L= length(obj.association);
n= n_L * params.m_F;
A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;
% ------------------------------------

end







