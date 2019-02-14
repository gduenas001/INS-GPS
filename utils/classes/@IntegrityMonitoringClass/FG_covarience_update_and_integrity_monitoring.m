
function FG_covarience_update_and_integrity_monitoring(obj, estimator, counters, data,  params)

% 
obj.Phi_k= estimator.Phi_k;
obj.H_k= estimator.H_k;
obj.D_bar_k= estimator.D_bar;
if counters.k_im > params.FG_prec_hor
    % compute extraction vector
    alpha= [zeros((params.FG_prec_hor+1)*params.m,1);...
           -sin(estimator.XX(params.ind_yaw));...
           cos(estimator.XX(params.ind_yaw));...
           0];
       
    % number of absolute msmts over the horizon
    obj.n= estimator.n_k + sum(obj.n_ph);
    
    % initialize normalized Jacobian 
    obj.A= zeros( obj.n + (params.FG_prec_hor+1) * (params.m-1) + params.m,...
                ( params.FG_prec_hor + 2 ) * params.m );
    obj.index_of_abs_msmt_in_A= []; %zeros(2, obj.n/2);
    obj.A(1:params.m,1:params.m)= sqrtm(eye(params.m)/(obj.PX_prior))*eye(params.m);
    r_ind= params.m + 1;
    r_col= 1;
    
    % sqrt of inverse of msmt cov matrix
    inv_V_1_2 = sqrtm(inv([params.sig_lidar^2,0;0,params.sig_lidar^2]));
    
    for i=1:params.FG_prec_hor+1
        if i==params.FG_prec_hor+1
            
            [~,S,V]=svd(obj.D_bar_k);
            r_S= rank(S);
            D_bar_k_p= sqrtm( eye(r_S) / S(1:r_S,1:r_S) ) * V(:,1:r_S)';
            
            obj.A(r_ind:r_ind+params.m-2,r_col:r_col-1+params.m)= ...
                D_bar_k_p * obj.Phi_k;
            
            obj.A(r_ind:r_ind+params.m-2,r_col+params.m:r_col-1+2*params.m)= ...
                -D_bar_k_p * eye(params.m);
            
            n_L_k= estimator.n_k/params.m_F;
            obj.A(r_ind+params.m-1:r_ind-2+params.m+estimator.n_k,r_col+params.m:r_col-1+2*params.m)= ...
                kron( eye( n_L_k ) ,inv_V_1_2 ) * obj.H_k;
            
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A,...
                reshape( r_ind+params.m-1 : r_ind-2+params.m+estimator.n_k , params.m_F , [] ) ];
            
        else
            
            [~,S,V]=svd(obj.D_bar_ph{params.FG_prec_hor-i+1});
            r_S= rank(S);
            D_bar_ph_p= ( sqrtm(eye(r_S) / S(1:r_S,1:r_S)) ) * V(:,1:r_S)';
            
            obj.A(r_ind:r_ind+params.m-2,r_col:r_col-1+params.m)= ...
                D_bar_ph_p * obj.Phi_ph{params.FG_prec_hor-i+1};
            
            obj.A(r_ind:r_ind+params.m-2,r_col+params.m:r_col-1+2*params.m)= ...
                -D_bar_ph_p*eye(params.m);
            
            n_L_ph= obj.n_ph(params.FG_prec_hor-i+1)/params.m_F;
            obj.A(r_ind+params.m-1:r_ind-2+params.m+obj.n_ph(params.FG_prec_hor-i+1) , r_col+params.m:r_col-1+2*params.m)= ...
                kron( eye(n_L_ph) , inv_V_1_2 ) * obj.H_ph{params.FG_prec_hor-i+1};
            
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A,...
                reshape( r_ind+params.m-1 : r_ind-2+params.m+obj.n_ph(params.FG_prec_hor-i+1) , params.m_F , [] ) ];
            
            r_ind= r_ind+params.m-1+obj.n_ph(params.FG_prec_hor-i+1);
            r_col= r_col+params.m;
            
        end
    end
    
    % initialization of p_hmi
    obj.p_hmi= 0;
    
    if ( rank(obj.A) < params.m*(params.FG_prec_hor+2) ) || (obj.n-(params.FG_prec_hor+1) < 6) % need at least 5 msmts (3 landmarks) to monitor one landmark fault
        fprintf('Rank of A is less than the size of the state vector: rank(A) = %d < %d\n', rank(obj.A), params.m*(params.FG_prec_hor+2))
        obj.p_hmi= 1;
    else % if we don't have enough landmarks --> P(HMI)= 1   
        obj.Gamma_FG= obj.A' * obj.A;
        
        if rank(obj.Gamma_FG) ~= params.m*(params.FG_prec_hor+2)
            a=0;
        end
        
        obj.PX_M_FG= eye(size(obj.Gamma_FG,1)) / obj.Gamma_FG;
        
        %if sum(diag(obj.PX_M_FG) < params.min_state_var_FG) > 0
        %    for i= 1:size(obj.PX_M_FG,1)
        %        if obj.PX_M_FG(i,i) < params.min_state_var_FG
        %            obj.PX_M_FG(i,i)= params.min_state_var_FG;
        %        end
        %    end
        %end
        
        obj.PX_prior= obj.PX_M_FG(params.m+1:2*params.m,params.m+1:2*params.m);
        
        %if sum(diag(obj.PX_prior) < params.min_state_var_FG) > 0
        %    for i= 1:size(obj.PX_prior,1)
        %        if obj.PX_prior(i,i) < params.min_state_var_FG
        %            obj.PX_prior(i,i)= params.min_state_var_FG;
        %        end
        %    end
        %end
        
        obj.M_FG= eye(size(obj.A,1)) - (obj.A / (obj.A' * obj.A)) * obj.A';
        
        obj.sigma_hat= sqrt( (alpha' / obj.Gamma_FG) * alpha );
        
        obj.n_H= obj.n/2;
        
        % set the threshold from the continuity req
        obj.detector_threshold= chi2inv( 1 - obj.C_req, obj.n-(params.FG_prec_hor+1) );
        
        % fault probability of each association in the preceding horizon
        obj.P_F_M= ones(obj.n/2,1) * params.P_UA;
        
        obj.P_H= ones(obj.n_H, 1) * inf; % initializing P_H vector
        
        for i= 0:obj.n_H
            % build extraction matrix
            if i == 0
                obj.compute_E_matrix_FG(params, 0, params.m_F);
            else
                obj.compute_E_matrix_FG(params, i, params.m_F);
            end
            
            if rank(obj.E * obj.M_FG * obj.E') < size(obj.E,1)
                obj.p_hmi= 1;
                break;
            end

            % Worst-case fault direction
            f_M_dir= obj.E' / (obj.E * obj.M_FG * obj.E') * obj.E * obj.A * obj.PX_M_FG * alpha;
            f_M_dir= f_M_dir / norm(f_M_dir); % normalize
            
            % worst-case fault magnitude
            fx_hat_dir= alpha' * obj.PX_M_FG * obj.A' * f_M_dir;
            M_dir= f_M_dir' * obj.M_FG * f_M_dir;
            
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
                % unfaulted_inds= all( 1:obj.n_L_M ~= fault_inds(i,:)', 1 );
                obj.P_H(i)= prod( obj.P_F_M( i ) ); %...
                % * prod( 1 - P_F_M(unfaulted_inds)  );
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