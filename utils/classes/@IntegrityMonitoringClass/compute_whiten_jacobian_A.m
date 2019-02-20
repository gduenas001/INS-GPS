
function compute_whiten_jacobian_A(obj, estimator, params)


% initialize normalized Jacobian
obj.A= zeros( obj.n_total, obj.m_M );

% indexes of the absolute measurements in the rows of matrix A
obj.abs_msmt_ind= [];

% plug the prior in A
obj.A( 1:params.m, 1:params.m )= sqrtm( inv(obj.PX_prior) );

% pointers to the next part of A to be filled
r_ind= params.m + 1;
c_ind= 1;

% sqrt of inverse of lidar msmt noise cov matrix
sqrt_inv_R_lidar = sqrtm( inv( params.R_lidar ) );

% build A whithen Jacobian
for i= 1:obj.M
    
    % gyro msmt submatrix
    obj.A( r_ind, c_ind : c_ind + params.m - 1 )= ...
        -params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    obj.A( r_ind, c_ind + params.m : c_ind + 2*params.m - 1 )= ...
        params.sig_gyro_z^(-1) * [0,0,1/params.dt_sim];
    
    % update the row index to point towards the next msmt
    r_ind= r_ind + 1;
    
    
    if i == obj.M
        
        % plug steering angle and wheel speed model in A
        [~,S,V]= svd( estimator.D_bar );
        r_S= rank(S);
        D_bar_p= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:, 1:r_S)';
        
        obj.A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1)= ...
            D_bar_p * estimator.Phi_k;
        
        obj.A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m - 1)= ...
            -D_bar_p;
        
        % update the row & column indexes
        r_ind= r_ind + r_S;
        c_ind= c_ind + params.m;
        
        % plug lidar model in A
        obj.A(  r_ind : r_ind + estimator.n_k - 1,...
            c_ind : c_ind + params.m - 1 )= ...
            kron( eye( estimator.n_L_k ) , sqrt_inv_R_lidar ) * estimator.H_k;
        
        % record lidar msmt indieces in A
        obj.abs_msmt_ind= [ obj.abs_msmt_ind,...
            reshape( r_ind : r_ind + estimator.n_k - 1, params.m_F , [] ) ];
        
    else
        
        % plug steering angle and wheel speed model in A
        [~,S,V]= svd( obj.D_bar_ph{ obj.M - i } );
        r_S= rank(S);
        D_bar_ph_p= sqrtm( inv(S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
        
        obj.A( r_ind : r_ind + r_S - 1, c_ind : c_ind + params.m - 1 )= ...
            D_bar_ph_p * obj.Phi_ph{ obj.M - i };
        
        obj.A( r_ind : r_ind + r_S - 1, c_ind + params.m : c_ind + 2*params.m -1)= ...
            -D_bar_ph_p;
        
        % update the row & column indexes
        r_ind= r_ind + r_S;
        c_ind= c_ind + params.m;
        
        % lidar Jacobian part
        n_L_i= obj.n_ph( obj.M - i ) / params.m_F;
        obj.A( r_ind : r_ind + obj.n_ph( obj.M - i ) - 1,...
            c_ind : c_ind + params.m - 1)= ...
            kron( eye(n_L_i) , sqrt_inv_R_lidar ) * obj.H_ph{ obj.M - i };
        
        % record lidar msmt indieces in A
        obj.abs_msmt_ind= [ obj.abs_msmt_ind,...
            reshape( r_ind : r_ind + obj.n_ph(obj.M-i) - 1, params.m_F , [] ) ];
        
        % update the row index
        r_ind= r_ind + obj.n_ph( obj.M - i );
    end
end


end





