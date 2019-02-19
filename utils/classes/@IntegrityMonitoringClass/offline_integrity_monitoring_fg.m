
function offline_integrity_monitoring_fg(obj, estimator, counters, data,  params)

if counters.k_im > obj.M
    % compute extraction vector
    alpha= [ zeros( obj.M * params.m, 1 );...
            -sin( estimator.x_true(params.ind_yaw) );...
             cos( estimator.x_true(params.ind_yaw) );...
             0 ];
       
    % number of absolute msmts over the horizon
    obj.n_M= estimator.n_k + sum( obj.n_ph(1:obj.M - 1) );
    
    % number of landmarks over the horizon
    obj.n_L_M= obj.n_M / params.m_F;
    
    % total number of msmts (prior + relative + abs)
    obj.n_total= obj.n_M + (obj.M + 1) * params.m;
    
    % number of states to estimate
    obj.m_M= (obj.M + 1) * params.m;
    
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
        
    % construct the information matrix
    obj.Gamma_fg= obj.A' * obj.A;
    
    % full covarince matrix
    obj.PX_M= inv(obj.Gamma_fg);
    
    % extract covarince matrix at time k
    estimator.PX= obj.PX_M( end - params.m + 1 : end, end - params.m + 1 : end );
    
    % find the prior covarince matrix for time k+1
    obj.PX_prior= obj.PX_M( params.m + 1 : 2*params.m, params.m + 1 : 2*params.m );
    
    
    % initialization of p_hmi
    obj.p_hmi=0;
    if obj.n_M < 5    
        % if we don't have enough landmarks --> P(HMI)= 1
        obj.p_hmi= 1;
        
    else % we have enough msmts
        
        % Least squares residual matrix
        obj.M_M= eye( obj.n_total ) - obj.A * obj.PX_M * obj.A';
        
        % standard deviation in the state of interest
        obj.sigma_hat= sqrt( (alpha' / obj.Gamma_fg) * alpha );
        
        % number of hypotheses (just one per abs msmt for now)
        obj.n_H= obj.n_L_M;
        
        % set detector threshold from the continuity req
        obj.T_d= chi2inv( 1 - obj.C_req, obj.n_M );
        
        % fault probability of each association in the preceding horizon
        obj.P_F_M= ones(obj.n_L_M, 1) * params.P_UA;
        
        % initializing P_H vector
        obj.P_H= ones(obj.n_H, 1) * inf;
        
        for i= 0:obj.n_H
            
            % build extraction matrix
            if i == 0
                obj.compute_E_matrix_fg( 0, params.m_F);
            else
                obj.compute_E_matrix_fg( i, params.m_F);
            end
            
            % Worst-case fault direction
            f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A * obj.PX_M * alpha;
            f_M_dir= f_M_dir / norm(f_M_dir); % normalize
            
            % worst-case fault magnitude
            fx_hat_dir= alpha' * obj.PX_M * obj.A' * f_M_dir;
            M_dir= f_M_dir' * obj.M_M * f_M_dir;
            
            % worst-case fault magnitude
            f_mag_min= 0;
            f_mag_max= 5;
            f_mag_inc= 5;
            p_hmi_H_prev= -1;
            for k= 1:10
                [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
                    f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
                    f_mag_min, f_mag_max);

                % make it a positive number
                p_hmi_H= -p_hmi_H;

                % check if the new P(HMI|H) is smaller
                if k == 1 || p_hmi_H_prev < p_hmi_H
                    p_hmi_H_prev= p_hmi_H;
                    f_mag_min= f_mag_min + f_mag_inc;
                    f_mag_max= f_mag_max + f_mag_inc;
                else
                    p_hmi_H= p_hmi_H_prev;
                    break
                end
            end
            
            
            % Add P(HMI | H) to the integrity risk
            if i == 0
                obj.P_H_0= prod( 1 - obj.P_F_M );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H_0;
            else
                obj.P_H(i)= prod( obj.P_F_M( i ) );
                obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H(i);
            end
        end
    end
    
    % store integrity related data
    data.store_integrity_data(obj, estimator, counters, params) 
end

% update the preceding horizon
update_preceding_horizon(obj, estimator, params)

end