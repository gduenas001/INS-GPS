
function FG_covarience_update_and_integrity_monitoring(obj, estimator, counters, data,  params)

if counters.k_im > params.FG_prec_hor
    % compute extraction vector
    alpha= [    zeros((params.FG_prec_hor+1)*params.m,1);...
                -sin(estimator.x_true(params.ind_yaw));...
                cos(estimator.x_true(params.ind_yaw));...
                0                                           ];
       
    % number of absolute msmts over the horizon
    obj.n= estimator.n_k + sum(obj.n_ph);
    
    % number of landmarks over the horizon
    obj.n_L_M= obj.n/2;

    
    % initialize normalized Jacobian 
    obj.A= zeros( obj.n + (params.FG_prec_hor+2) * (params.m),...
                        ( params.FG_prec_hor + 2 ) * params.m );
    obj.index_of_abs_msmt_in_A= [];
    
    % plug the prior in A
    obj.A(1:params.m,1:params.m)= sqrtm(eye(params.m)/(obj.PX_prior))*eye(params.m);
    
    % pointers to the next part of A to be filled
    r_ind= params.m + 1;
    r_col= 1;
    
    % sqrt of inverse of lidar msmt noise cov matrix
    inv_V_1_2 = sqrtm(inv([params.sig_lidar^2,0;0,params.sig_lidar^2]));
    
    for i=1:params.FG_prec_hor+1
        if i==params.FG_prec_hor+1
            

            
            % plug Gyro(z) model in A
            obj.A(r_ind,r_col:r_col-1+params.m)= ...
                    params.P_Gyro_z^(-1/2) * [0,0,1/params.dt_sim];
        
            obj.A(r_ind,r_col+params.m:r_col-1+2*params.m)= ...
            params.P_Gyro_z^(-1/2) * [0,0,-1/params.dt_sim];
        
        

            % plug steering angle and wheel speed model in A
            [~,S,V]=svd(estimator.D_bar);
            r_S= rank(S);
            D_bar_p= sqrtm( eye(r_S) / S(1:r_S,1:r_S) ) * V(:,1:r_S)';
            
            obj.A(r_ind+1:r_ind+params.m-1,r_col:r_col-1+params.m)= ...
                D_bar_p * estimator.Phi_k;
            
            obj.A(r_ind+1:r_ind+params.m-1,r_col+params.m:r_col-1+2*params.m)= ...
                -D_bar_p * eye(params.m);
            
            
            
            % plug lidar model in A
            obj.A(r_ind+params.m:r_ind-1+params.m+estimator.n_k,r_col+params.m:r_col-1+2*params.m)= ...
                    kron( eye( estimator.n_L_k ) ,inv_V_1_2 ) * estimator.H_k;
            
            
            % record lidar msmt indieces in A
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A,...
                reshape( r_ind+params.m : r_ind-1+params.m+estimator.n_k , params.m_F , [] ) ];
            
        else
            
            
            % plug Gyro(z) model in A
            obj.A(r_ind,r_col:r_col-1+params.m)= ...
            params.P_Gyro_z^(-1/2) * [0,0,1/params.dt_sim];
        
            obj.A(r_ind,r_col+params.m:r_col-1+2*params.m)= ...
            params.P_Gyro_z^(-1/2) * [0,0,-1/params.dt_sim];
        
        
        
            % plug steering angle and wheel speed model in A
            [~,S,V]=svd(obj.D_bar_ph{params.FG_prec_hor-i+1});
            r_S= rank(S);
            D_bar_ph_p= ( sqrtm(eye(r_S) / S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
            
            obj.A(r_ind+1:r_ind+params.m-1,r_col:r_col-1+params.m)= ...
                D_bar_ph_p * obj.Phi_ph{params.FG_prec_hor-i+1};
            
            obj.A(r_ind+1:r_ind+params.m-1,r_col+params.m:r_col-1+2*params.m)= ...
                -D_bar_ph_p*eye(params.m);
            
            
            
            % plug lidar model in A
            n_L_ph= obj.n_ph(params.FG_prec_hor-i+1)/params.m_F;
            obj.A(r_ind+params.m:r_ind-1+params.m+obj.n_ph(params.FG_prec_hor-i+1) , r_col+params.m:r_col-1+2*params.m)= ...
                kron( eye(n_L_ph) , inv_V_1_2 ) * obj.H_ph{params.FG_prec_hor-i+1};
            
            
            % record lidar msmt indieces in A
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A,...
                reshape( r_ind+params.m : r_ind-1+params.m+obj.n_ph(params.FG_prec_hor-i+1) , params.m_F , [] ) ];
            
            
            
            % update pointers to the next part of A to be filled
            r_ind= r_ind+params.m+obj.n_ph(params.FG_prec_hor-i+1);
            r_col= r_col+params.m;
            
        end
    end
    
    
    % initialization of p_hmi
    obj.p_hmi=0;
    
    
    % need at least 5 msmts (3 landmarks) to monitor one landmark fault
    if (obj.n < 5)
        
        fprintf('Rank of A is less than the size of the state vector: rank(A) = %d < %d\n', rank(obj.A), params.m*(params.FG_prec_hor+2))
        
        
        % construct the information matrix
        obj.Gamma_FG= obj.A' * obj.A;
        
        
        % find the full covarince matrix
        obj.PX_M= inv(obj.Gamma_FG);
        
        
        % find the covarince matrix at time k
        estimator.PX= obj.PX_M(end-params.m+1:end,end-params.m+1:end);
        
        
        % find the prior covarince matrix for time k+1
        obj.PX_prior= obj.PX_M(params.m+1:2*params.m,params.m+1:2*params.m);
        
        % if we don't have enough landmarks --> P(HMI)= 1
        obj.p_hmi= 1;
        
    else
        
        % construct the information matrix
        obj.Gamma_FG= obj.A' * obj.A;
        
        
        % find the full covarince matrix
        obj.PX_M= eye(size(obj.Gamma_FG,1)) / obj.Gamma_FG;
        
        
        % find the covarince matrix at time k
        estimator.PX= obj.PX_M(end-params.m+1:end,end-params.m+1:end);
        
        
        % find the prior covarince matrix for time k+1
        obj.PX_prior= obj.PX_M(params.m+1:2*params.m,params.m+1:2*params.m);
        
        
        % Least squares residual matrix
        obj.M_M= eye(size(obj.A,1)) - obj.A * obj.PX_M * obj.A';
        
        
        % standard deviation in the state of interest
        obj.sigma_hat= sqrt( (alpha' / obj.Gamma_FG) * alpha );
        
        
        % monitoring the comb of single LM faults
        obj.n_H= obj.n_L_M;
        
        
        % set the threshold from the continuity req
        obj.detector_threshold= chi2inv( 1 - obj.C_req, obj.n-(params.FG_prec_hor+1) );
        
        
        % fault probability of each association in the preceding horizon
        obj.P_F_M= ones(obj.n_L_M,1) * params.P_UA;
        
        
        % initializing P_H vector
        obj.P_H= ones(obj.n_H, 1) * inf;
        
        
        for i= 0:0%obj.n_H
            
            % build extraction matrix
            if i == 0
                obj.compute_E_matrix_FG(params, 0, params.m_F);
            else
                obj.compute_E_matrix_FG(params, i, params.m_F);
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
                    f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n-(params.FG_prec_hor+1) ),...
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

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
update_preceding_horizon(obj, estimator, params)

end