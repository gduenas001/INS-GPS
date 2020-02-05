function prob_of_MA(obj, estimator, params)


% dof of the non-central chi-square in the P(MA)
chi_dof= params.m_F;

% allocate memory
spsi= sin(estimator.x_true(params.ind_yaw));
cpsi= cos(estimator.x_true(params.ind_yaw));
h_t= zeros(2,1);
h_l= zeros(2,1);
%estimator.association_no_zeros= estimator.association( estimator.association ~= 0);
%obj.P_MA_k= ones(size(estimator.association_no_zeros)) * (-1);
obj.P_MA_k= ones(estimator.n_L_k,1) * (-1);
obj.P_MA_k_full= obj.P_MA_k;

%---------------Osama---------------------
obj.P_MA_k = obj.P_MA_k*0;
obj.P_MA_k_full = obj.P_MA_k_full*0;
return;
%-----------------------------------------

% loop through each associated landmark
for t= 1:estimator.n_L_k
    % take the landmark ID
    lm_id_t= estimator.lm_ind_fov(t);
    
    % initialize the P(MA)
    obj.P_MA_k(t)= params.I_MA + (length(estimator.lm_ind_fov) - 1)*(1-params.I_MA);
    
    % build the necessary parameters
    landmark= estimator.landmark_map( lm_id_t ,: );
    dx= landmark(1) - estimator.x_true(1);
    dy= landmark(2) - estimator.x_true(2);
    h_t(1)=  dx*cpsi + dy*spsi;
    h_t(2)= -dx*spsi + dy*cpsi;
    R_it=[];
    % compute kappa
    if isempty(obj.A)
        obj.mu_k = 0;
%    elseif obj.n_M < params.m + obj.n_max*params.m_F
%        obj.kappa= 1;
%        obj.G_l_t= obj.kappa * ( sqrt(obj.T_d) - sqrt( chi2inv(1 - params.I_MA , obj.n_M) ) )^2;
    else
        obj.kappa= 0;
        % compute Q matrix with A_M_(k-1) , Phi_(k-1), P_k, n_M_(k-1)
        H_t= zeros( params.m_F , obj.m_M );
        H_t(:,end-params.m+1)= [-cpsi;spsi];
        H_t(:,end-params.m+2)= [-spsi;-cpsi];
        H_t(:,end-params.m+params.ind_yaw)= [-dx*spsi + dy*cpsi;-dx*cpsi - dy*spsi ];
        A_t= params.R_lidar\H_t;
        E_i= [zeros(params.m_F,obj.n_total-estimator.n_k),zeros(params.m_F,(t-1)*params.m_F),eye(params.m_F),zeros(params.m_F,estimator.n_k-t*params.m_F)];
        R_it= (E_i-A_t*obj.PX_M*obj.A')*(E_i-A_t*obj.PX_M*obj.A')';
        Q= obj.A * obj.PX_M *((A_t')/R_it)*A_t* obj.PX_M *obj.A';
        for j= 1:( min(obj.n_max,obj.n_L_M) )%(obj.n_L_M-3)
            C = nchoosek(1:obj.n_L_M,j); %set of possible fault indices for n_max simultanous faults
            for i= 1:size(C,1)
                % build extraction matrix
                obj.compute_E_matrix_fg(C(i,:), params.m_F);
                %kappa_H= eigs( obj.E*Q*obj.E', 1) * eigs( obj.E*obj.M_M*obj.E', 1, 'smallestabs');
                N = obj.E*(eye(obj.n_total)-(obj.A*obj.PX_M*(obj.A')))*(obj.E');
                kappa_H= eigs( sqrtm(N)\(obj.E*Q*obj.E')/sqrtm(N), 1);
                % take the largest kappa
                if kappa_H > obj.kappa, obj.kappa= kappa_H; end
            end
        end
        obj.G_l_t= obj.kappa * ( sqrt(obj.T_d) - sqrt( chi2inv(1 - params.I_MA , obj.n_M) ) )^2;
    end
    
    % loop through every possible landmark in the FoV (potential MA)
    for l= 1:length(estimator.lm_ind_fov)
        % take landmark ID
        lm_id_l= estimator.lm_ind_fov(l);
        if lm_id_t ~= lm_id_l
            
            % extract the landmark
            landmark= estimator.landmark_map( lm_id_l ,: );
            
            % compute necessary intermediate parameters
            dx= landmark(1) - estimator.x_true(1);
            dy= landmark(2) - estimator.x_true(2);
            h_l(1)=  dx*cpsi + dy*spsi;
            h_l(2)= -dx*spsi + dy*cpsi;
            %H_l= [-cpsi, -spsi, -dx*spsi + dy*cpsi;...
            %       spsi, -cpsi, -dx*cpsi - dy*spsi ];
            y_l_t= sqrtm(params.R_lidar)\(h_l - h_t);
            %Y_l= H_l * estimator.PX(params.ind_pose, params.ind_pose) * H_l' + params.R_lidar;
            
            % individual innovation norm between landmarks l and t
            if isempty(R_it)
                IIN_l_t= 0;
            else
                IIN_l_t= sqrt( (y_l_t'/R_it )* y_l_t );
            end
            % if one of the landmarks is too close to ensure P(MA) --> set to one
            if IIN_l_t < sqrt(params.T_NN)
                obj.P_MA_k(t)= 1;
                break
            else
                obj.P_MA_k(t)= obj.P_MA_k(t) -...
                    (ncx2cdf( ( IIN_l_t - sqrt(params.T_NN) )^2 , chi_dof, obj.G_l_t ))*(1-params.I_MA);
            end            
        end
    end
    
    % store the P_MA for the full LMs
    obj.P_MA_k_full(t)= obj.P_MA_k(t);
    
    % landmark selection
    %if params.SWITCH_LM_SELECTION
    %    if obj.P_MA_k(t) > params.P_MA_max
    %        obj.P_MA_k(t)= -1;
    %        estimator.association_no_zeros(t)= -1;
    %        estimator.association( estimator.association == lm_id_t )= 0;
    %    end
    %end
    
    % not more than probability one
    if obj.P_MA_k(t) > 1, obj.P_MA_k(t)= 0.99999999999999; end
end

% remove non-associated ones
%if params.SWITCH_LM_SELECTION
%    obj.P_MA_k( obj.P_MA_k == -1 )= [];
%    estimator.association_no_zeros( estimator.association_no_zeros == -1 )= [];
%end

end