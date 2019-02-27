
function compute_whiten_jacobian_A_exp(obj, estimator, params)
% this function computes the A jacobian for future use in integrity monitoring

% initialize normalized Jacobian
obj.A= zeros( obj.n_total, obj.m_M );

% indices of the absolute measurements in the rows of matrix A
obj.abs_msmt_ind= [];
obj.gps_msmt_ind= [];

% plug the prior in A
obj.A( 1:params.m, 1:params.m )= sqrtm( obj.Gamma_prior );

% pointers to the next part of A to be filled
r_ind= params.m + 1;
c_ind= 1;

% build A whithen Jacobian
for i= obj.M-1 : -1 : 0
    
    if i == 0
        
        % plug whitened IMU model in A
        obj.A( r_ind : r_ind + params.m - 1, c_ind : c_ind + params.m - 1)= ...
            sqrtm( inv(estimator.D_bar) ) * estimator.Phi_k;
        
        obj.A( r_ind : r_ind + params.m - 1, c_ind + params.m : c_ind + 2*( params.m) - 1)= ...
            -sqrtm( inv(estimator.D_bar) );
        
        % update the row & column indices
        r_ind= r_ind + params.m;
        c_ind= c_ind + params.m;
        
        
        % plug whitened gps model in A
        if estimator.n_gps_k ~= 0
            
            obj.A(  r_ind : r_ind + estimator.n_gps_k - 1,...
                c_ind : c_ind + params.m - 1 )= ...
                estimator.H_k_gps;
            
            % record gps msmt indieces in A
            obj.gps_msmt_ind= [ obj.gps_msmt_ind,...
                (r_ind : r_ind + estimator.n_gps_k - 1)' ];
            
            % update the row indices
            r_ind= r_ind + estimator.n_gps_k;
            
        end
        
        
        % plug whitened lidar model in A
        obj.A(  r_ind : r_ind + estimator.n_k - 1,...
            c_ind : c_ind + params.m - 1 )= ...
            estimator.H_k_lidar;
        
        % record lidar msmt indieces in A
        obj.abs_msmt_ind= [ obj.abs_msmt_ind,...
            reshape( r_ind : r_ind + estimator.n_k - 1, params.m_F , [] ) ];
        
        
        
    else
        
        % plug whitened IMU model in A
        obj.A( r_ind : r_ind + params.m - 1, c_ind : c_ind + params.m - 1)= ...
            sqrtm( inv(obj.D_bar_ph{ i + 1 }) ) * obj.Phi_ph{ i + 1 };
        
        obj.A( r_ind : r_ind + params.m - 1, c_ind + params.m : c_ind + 2*(params.m) - 1)= ...
            -sqrtm( inv(obj.D_bar_ph{ i + 1 }) );
        
        % update the row & column indices
        r_ind= r_ind + params.m;
        c_ind= c_ind + params.m;

        % plug whitened gps model in A
        if obj.n_gps_ph( i ) ~= 0
            
            obj.A(  r_ind : r_ind + obj.n_gps_ph( i ) - 1,...
                c_ind : c_ind + params.m - 1 )= ...
                obj.H_gps_ph{ i };
            
            % record gps msmt indieces in A
            obj.gps_msmt_ind= [ obj.gps_msmt_ind,...
                (r_ind : r_ind + obj.n_gps_ph( i ) - 1)' ];
            
            % update the row indices
            r_ind= r_ind + obj.n_gps_ph( i );
            
        end
        
        % lidar Jacobian part
        n_L_i= obj.n_ph( i ) / params.m_F;
        obj.A( r_ind : r_ind + obj.n_ph( i ) - 1,...
            c_ind : c_ind + params.m - 1 )= ...
            obj.H_lidar_ph{ i };
        
        % record lidar msmt indieces in A
        obj.abs_msmt_ind= [ obj.abs_msmt_ind,...
            reshape( r_ind : r_ind + obj.n_ph(i) - 1, params.m_F , [] ) ];
        
        % update the row index
        r_ind= r_ind + obj.n_ph( i );
    end
end


end





