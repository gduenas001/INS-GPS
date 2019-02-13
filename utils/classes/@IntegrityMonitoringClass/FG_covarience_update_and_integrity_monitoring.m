
function FG_covarience_update_and_integrity_monitoring(obj, estimator, counters, data,  params)
obj.Phi_k= estimator.Phi_k;
obj.H_k= estimator.H_k;
obj.D_bar_k= estimator.D_bar;
if counters.k_im > params.FG_preceding_horizon_in_epochs
    alpha= [zeros((params.FG_preceding_horizon_in_epochs+1)*params.m,1) ;-sin(estimator.XX(params.ind_yaw)); cos(estimator.XX(params.ind_yaw)); 0];
    obj.n= estimator.n_k + sum(obj.n_ph);
    obj.A=zeros(obj.n + (params.FG_preceding_horizon_in_epochs+1)...
        *(params.m-1) + params.m,(params.FG_preceding_horizon_in_epochs+2)*params.m);
    obj.index_of_abs_msmt_in_A= []; %zeros(2, obj.n/2);
    obj.A(1:params.m,1:params.m)= sqrtm(eye(params.m)/(obj.PX_prior))*eye(params.m);
    dummy_var_rows= params.m + 1;
    dummy_var_columns= 1;
    for i=1:params.FG_preceding_horizon_in_epochs+1
        if i==params.FG_preceding_horizon_in_epochs+1
            [~,S,V]=svd(obj.D_bar_k);
            D_bar_k_p= (sqrtm(eye(rank(S))/S(1:rank(S),1:rank(S))))*V(:,1:rank(S))';
            obj.A(dummy_var_rows:dummy_var_rows+params.m-2,dummy_var_columns:dummy_var_columns-1+params.m)= ...
                D_bar_k_p*obj.Phi_k;
            obj.A(dummy_var_rows:dummy_var_rows+params.m-2,dummy_var_columns+params.m:dummy_var_columns-1+2*params.m)= ...
                D_bar_k_p*eye(params.m);
            obj.A(dummy_var_rows+params.m-1:dummy_var_rows-2+params.m+estimator.n_k,dummy_var_columns+params.m:dummy_var_columns-1+2*params.m)= ...
                kron(eye(estimator.n_k/params.m_F),sqrtm(inv([params.sig_lidar^2,0;0,params.sig_lidar^2])))*obj.H_k;
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A, reshape( dummy_var_rows+params.m-1:dummy_var_rows-2+params.m+estimator.n_k ,params.m_F,[])];
        else
            [~,S,V]=svd(obj.D_bar_ph{params.FG_preceding_horizon_in_epochs-i+1});
            D_bar_ph_p= (sqrtm(eye(rank(S))/S(1:rank(S),1:rank(S))))*V(:,1:rank(S))';
            obj.A(dummy_var_rows:dummy_var_rows+params.m-2,dummy_var_columns:dummy_var_columns-1+params.m)= ...
                D_bar_ph_p*obj.Phi_ph{params.FG_preceding_horizon_in_epochs-i+1};
            obj.A(dummy_var_rows:dummy_var_rows+params.m-2,dummy_var_columns+params.m:dummy_var_columns-1+2*params.m)= ...
                D_bar_ph_p*eye(params.m);
            obj.A(dummy_var_rows+params.m-1:dummy_var_rows-2+params.m+obj.n_ph(params.FG_preceding_horizon_in_epochs-i+1),dummy_var_columns+params.m:dummy_var_columns-1+2*params.m)= ...
                kron(eye(obj.n_ph(params.FG_preceding_horizon_in_epochs-i+1)/params.m_F),sqrtm(inv([params.sig_lidar^2,0;0,params.sig_lidar^2])))*obj.H_ph{params.FG_preceding_horizon_in_epochs-i+1};
            obj.index_of_abs_msmt_in_A= [ obj.index_of_abs_msmt_in_A, reshape( dummy_var_rows+params.m-1:dummy_var_rows-2+params.m+obj.n_ph(params.FG_preceding_horizon_in_epochs-i+1) ,params.m_F,[])];
            dummy_var_rows= dummy_var_rows+params.m-1+obj.n_ph(params.FG_preceding_horizon_in_epochs-i+1);
            dummy_var_columns= dummy_var_columns+params.m;
        end
    end
    
    % initialization of p_hmi
    obj.p_hmi= 0;
    
    if ( rank(obj.A) < params.m*(params.FG_preceding_horizon_in_epochs+2) ) % need at least 5 msmts (3 landmarks) to monitor one landmark fault
        fprintf('Rank of A is less than the size of the state vector: rank(A) = %d < %d\n', rank(obj.A), params.m*(params.FG_preceding_horizon_in_epochs+2))
        obj.p_hmi= 1;
    else % if we don't have enough landmarks --> P(HMI)= 1   
        obj.Gamma_FG= obj.A' * obj.A;
        obj.PX_hor_FG= eye(size(obj.Gamma_FG,1)) / obj.Gamma_FG;
        obj.PX_prior= obj.PX_hor_FG(params.m+1:2*params.m,params.m+1:2*params.m);
        obj.M_FG= eye(size(obj.A,1)) - obj.A * obj.PX_hor_FG * obj.A';
        
        obj.n_H= obj.n/2;
        %obj.P_H= ones(obj.n_H, 1) * inf; % initializing P_H vector
        
        for i= 0:obj.n_H
            % build extraction matrix
            if i == 0
                obj.compute_E_matrix_FG(params, 0, params.m_F);
            else
                obj.compute_E_matrix_FG(params, i, params.m_F);
            end

            % Worst-case fault direction
            f_M_dir= obj.E' / (obj.E * obj.M_FG * obj.E') * obj.E * obj.A * obj.PX_hor_FG * alpha;
            f_M_dir= f_M_dir / norm(f_M_dir); % normalize
        end
        
    end
    
end

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
update_preceding_horizon(obj, estimator, params)

end