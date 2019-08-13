
function [cost, grad, hessian, A, b]= optimization_fn(obj, x, params, k)

% previous x star
x_prev= obj.from_estimator_to_vector(params);

% from a vector to cells
obj.from_vector_to_estimator(x, params)

% compute delta
delta= x - x_prev;

% initialize
cost= 0;
inv_R_lidar= inv(params.R_lidar);
sqrt_inv_R_lidar= sqrtm(inv_R_lidar);

if k == 0
    
    % cost of the prior msmt
    z= obj.x_prior;
    z_expected= x(1:params.m);
    innov= z - z_expected;
    inv_PX_prior= inv(obj.PX_prior);
    cost= cost +  innov' * inv_PX_prior * innov;
    if nargout > 1

        sqrt_inv_PX_prior= sqrtm(inv_PX_prior);

        % initialize normalized Jacobian
        A= zeros( obj.n_total, obj.m_M );

        % plug the prior in A
        A( 1:params.m, 1:params.m )= sqrt_inv_PX_prior;

        % initialize b and add the prior residual
        b= ones(obj.n_total,1) * inf;
        b( 1:params.m )= sqrt_inv_PX_prior * innov;
    end

    % pointers to the next part of A to be filled
    r_ind= params.m + 1;
    c_ind= 1;

else
    
    A= zeros( obj.n_total-params.m_F-params.m, obj.m_M );
    b= ones( obj.n_total-params.m_F-params.m, 1 ) * inf;
    
    % pointers to the next part of A to be filled
    r_ind= 1;
    c_ind= 1;
    
end

for i= obj.M:-1:2
    
    % ------------ gyro cost ------------
    z= obj.z_gyro_ph{i};
    z_expected= pi_to_pi(( obj.x_ph{i-1}(params.ind_yaw) - obj.x_ph{i}(params.ind_yaw) ))...
        / params.dt_sim;
    innov= pi_to_pi(z - z_expected);
    cost= cost + innov^2 / params.sig_gyro_z^2;
    % ------------------------------------

    % ---------- gyro submatrix ----------
    if nargout > 1
        A( r_ind, c_ind : c_ind + params.m - 1 )= ...
            -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
        
        A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
            params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
        
        b( r_ind )= innov / params.sig_gyro_z;
    end
    % ------------------------------------
    
    
    % update the row index to point towards the next msmt
    r_ind= r_ind + 1;
    
    
    % ---------- odometry cost ----------
    z_expected= obj.return_odometry_update(obj.x_ph{i}, obj.odometry_ph{i}, params) - obj.x_ph{i-1};
    z_expected(params.ind_yaw)= pi_to_pi(z_expected(params.ind_yaw));
    innov= - z_expected;
    [Phi, D_bar]= obj.return_Phi_and_D_bar(...
        obj.x_ph{i}, obj.odometry_ph{i}(1), obj.odometry_ph{i}(2), params);
    [~,S,V]= svd( D_bar );
    r_S= rank(S);
    sqrt_inv_D_bar= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
%     cost= cost + z_expected' * pinv(D_bar) * z_expected;
    cost= cost + innov' * (sqrt_inv_D_bar' * sqrt_inv_D_bar) * innov;
    % ------------------------------------
    
    % ---------- odometry submatrix ----------
    if nargout > 1
        A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1 )= ...
            sqrt_inv_D_bar * Phi;
        
        A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m -1)= ...
            -sqrt_inv_D_bar;
        
        b( r_ind : r_ind + r_S - 1)= sqrt_inv_D_bar * innov;
    end
    % ------------------------------------

    
    % update the row & column indexes
    r_ind= r_ind + r_S;
    c_ind= c_ind + params.m;
    
    if i == obj.M
        tmp= length(obj.z_lidar_ph{i-1})/params.m_F;
    else
        tmp= tmp + length(obj.z_lidar_ph{i-1})/params.m_F;
    end
    
    if (k > 0) && (tmp-(length(obj.z_lidar_ph{i-1})/params.m_F) < k) && (tmp>=k)
        
        % ------------ lidar cost ------------
        ind_of_faulted_LM= k - (tmp-(length(obj.z_lidar_ph{i-1})/params.m_F));
        z= obj.z_lidar_ph{i-1};
        z((ind_of_faulted_LM-1)*params.m_F+1:ind_of_faulted_LM*params.m_F)=[];
        n= length(z);
        n_L= n / params.m_F;
        if n > 0
            z_expected= obj.return_expected_z_lidar(obj.x_ph{i-1}, obj.association_ph{i-1}, params, ind_of_faulted_LM);
            innov= z - z_expected;
            kron_inv_R_lidar= kron( eye(n_L) , inv_R_lidar );
            kron_sqrt_inv_R_lidar=  kron( eye(n_L) , sqrt_inv_R_lidar );
            cost= cost + innov' *  kron_inv_R_lidar * innov;
            % ------------------------------------

            % ---------- lidar submatrix ----------
            if nargout > 1
                A_lidar= obj.return_lidar_A(obj.x_ph{i-1}, obj.association_ph{i-1}, params, ind_of_faulted_LM);
                A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;

                b( r_ind : r_ind + n - 1)= kron_sqrt_inv_R_lidar * innov;
            end
            % ------------------------------------
        end

        % update the row index
        r_ind= r_ind + n;
        
    else
        
        % ------------ lidar cost ------------
        z= obj.z_lidar_ph{i-1};
        n= length(z);
        n_L= n / params.m_F;
        if n > 0
            z_expected= obj.return_expected_z_lidar(obj.x_ph{i-1}, obj.association_ph{i-1}, params, 0);
            innov= z - z_expected;
            kron_inv_R_lidar= kron( eye(n_L) , inv_R_lidar );
            kron_sqrt_inv_R_lidar=  kron( eye(n_L) , sqrt_inv_R_lidar );
            cost= cost + innov' *  kron_inv_R_lidar * innov;
            % ------------------------------------

            % ---------- lidar submatrix ----------
            if nargout > 1
                A_lidar= obj.return_lidar_A(obj.x_ph{i-1}, obj.association_ph{i-1}, params, 0);
                A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;

                b( r_ind : r_ind + n - 1)= kron_sqrt_inv_R_lidar * innov;
            end
            % ------------------------------------
        end

        % update the row index
        r_ind= r_ind + n;
        
    end
end


% ------------ gyro cost ------------
z= obj.z_gyro;
z_expected= ( pi_to_pi(obj.XX(params.ind_yaw) - obj.x_ph{1}(params.ind_yaw) )) / params.dt_sim;
innov= pi_to_pi(z - z_expected);
cost= cost + innov^2 / params.sig_gyro_z^2;
% ------------------------------------

% ---------- gyro submatrix ----------
if nargout > 1
    A( r_ind, c_ind : c_ind + params.m - 1 )= ...
        -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
        params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    b(r_ind)= innov / params.sig_gyro_z;
end
% ------------------------------------


% update the row index to point towards the next msmt
r_ind= r_ind + 1;
 
    
% ---------- odometry cost ----------
z_expected= obj.return_odometry_update(obj.x_ph{1}, obj.odometry_k, params) - obj.XX;
z_expected(params.ind_yaw)= pi_to_pi(z_expected(params.ind_yaw));
innov= - z_expected;
[Phi, D_bar]= obj.return_Phi_and_D_bar(...
    obj.x_ph{1}, obj.odometry_k(1), obj.odometry_k(2), params);
[~,S,V]= svd( D_bar );
r_S= rank(S);
sqrt_inv_D_bar= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
% cost= cost + z_expected' * pinv(D_bar) * z_expected;
cost= cost + innov' * (sqrt_inv_D_bar' * sqrt_inv_D_bar) * innov;
% ------------------------------------

% ---------- odometry submatrix ----------
if nargout > 1
    A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1 )= ...
        sqrt_inv_D_bar * Phi;
    
    A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m -1)= ...
        -sqrt_inv_D_bar;
    
    b( r_ind : r_ind + r_S - 1 )= sqrt_inv_D_bar * innov;
end
% ------------------------------------


% update the row & column indexes
r_ind= r_ind + r_S;
c_ind= c_ind + params.m;

% ------------ lidar cost ------------
z= obj.z_lidar';
z= z(:);
tmp= tmp + length(z)/params.m_F;

if (k > 0) && (tmp-(length(z)/params.m_F) < k) && (tmp>=k)
    
    ind_of_faulted_LM= k - (tmp-(length(z)/params.m_F));
    z((ind_of_faulted_LM-1)*params.m_F+1:ind_of_faulted_LM*params.m_F)=[];
    n= length(z);
    n_L= n / params.m_F;
    
    if n > 0
        z_expected= obj.return_expected_z_lidar(obj.XX, obj.association, params, ind_of_faulted_LM);
        innov= z - z_expected;
        kron_inv_R_lidar= kron( eye(n_L) , inv_R_lidar );
        kron_sqrt_inv_R_lidar=  kron( eye(n_L) , sqrt_inv_R_lidar );
        cost= cost + innov' *  kron_inv_R_lidar * innov;
        % ------------------------------------

        % ---------- lidar submatrix ----------
        if nargout > 1
            A_lidar= obj.return_lidar_A(obj.XX, obj.association, params, ind_of_faulted_LM);
            A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;

            b(r_ind : r_ind + n - 1)= kron_sqrt_inv_R_lidar * innov;
        end
        % ------------------------------------
    end
else
    
    n= length(z);
    n_L= n / params.m_F;
    
    if n > 0
        z_expected= obj.return_expected_z_lidar(obj.XX, obj.association, params, 0);
        innov= z - z_expected;
        kron_inv_R_lidar= kron( eye(n_L) , inv_R_lidar );
        kron_sqrt_inv_R_lidar=  kron( eye(n_L) , sqrt_inv_R_lidar );
        cost= cost + innov' *  kron_inv_R_lidar * innov;
        % ------------------------------------

        % ---------- lidar submatrix ----------
        if nargout > 1
            A_lidar= obj.return_lidar_A(obj.XX, obj.association, params, 0);
            A( r_ind : r_ind + n - 1, c_ind : c_ind + params.m - 1)= A_lidar;

            b(r_ind : r_ind + n - 1)= kron_sqrt_inv_R_lidar * innov;
        end
        % ------------------------------------
    end
end

if nargout > 1
    % return hessian
    hessian= A'*A;
    
    % return the gradient
    grad= hessian * delta - A' * b;
    
end

% divide cost by 2 to make Jacobian consistent
cost= cost / 2;

end

