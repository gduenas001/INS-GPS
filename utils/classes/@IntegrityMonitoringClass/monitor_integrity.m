function monitor_integrity(obj, estimator, counters, data, params)

% keep only the elements for the [x-y-theta]
obj.PX= estimator.PX(obj.ind_im, obj.ind_im);
obj.Phi_k= estimator.Phi_k^12; obj.Phi_k= obj.Phi_k( obj.ind_im, obj.ind_im ); %%%%%%%% CAREFUL
obj.H_k= estimator.H_k(:, obj.ind_im);
obj.L_k= estimator.L_k(obj.ind_im,:);

% if it's only 1 --> cannot compute Lpp_k
if counters.k_im > 1
    obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
else
    obj.Lpp_k= 0;
end

% monitor integrity if there are enough epochs
if counters.k_im > obj.M + 1 % need an extra epoch to store Lpp
    
    % common parameters
    obj.n_M= sum( obj.n_ph ) + estimator.n_k;
    obj.n_L_M= obj.n_M / params.m_F;
    alpha= [-sin(estimator.XX(3)); cos(estimator.XX(3)); zeros(obj.m-2,1)];
    
    
    % Update the innovation vector covarience matrix for the new PH
    obj.compute_Y_M_matrix(estimator)
    
    % compute the A matrix for the preceding horizon
    obj.compute_A_M_matrix()
        
    % compute B_bar matrix
    obj.compute_B_bar_matrix(estimator)
    
    % compute inverse of Y_M for future use
    invY_M= inv(obj.Y_M);
    
    % M matrix
    obj.M_M= obj.B_bar' * invY_M * obj.B_bar;
    
    % set the threshold from the continuity req
    obj.detector_threshold= chi2inv(1 - obj.C_req, obj.n_M);
    
    % compute detector
    obj.gamma_M= [ estimator.gamma_k ; cell2mat(obj.gamma_ph') ];
    obj.q_k= obj.gamma_M' * invY_M * obj.gamma_M;
    
    % Loop over hypotheses in the PH (only 1 fault)
    n_H= obj.n_M / params.m_F; % one hypothesis per associated landmark in ph
    obj.p_hmi= 0;
    for i= 0:n_H
        % build extraction matrix 
        obj.compute_E_matrix(i, params.m_F)
        
        % Worst-case fault direction
        f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A_M' * alpha;
        f_M_dir= f_M_dir / norm(f_M_dir); % normalize
        
        % worst-case fault magnitude
        sigma_hat= sqrt(alpha' * obj.PX * alpha);
        fx_hat_dir= alpha' * obj.A_M * f_M_dir;
        M_dir= f_M_dir' * obj.M_M * f_M_dir;
        
%         [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag)...
%             -((1-   normcdf(params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat)   +...
%             normcdf(-params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat))...
%             * ncx2cdf(obj.detector_threshold, params.m_F * obj.n_L_M, f_M_mag^2 * M_dir )),...
%             -10, 10);
        
        [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
            f_M_mag, fx_hat_dir, M_dir, sigma_hat, params.alert_limit, params.m_F * obj.n_L_M),...
            -10, 10);
        
        
        p_hmi_H= -p_hmi_H; % make it a positive number
        
        % Add P(HMI | H) to the integrity risk
        if i == 0
            obj.p_hmi= obj.p_hmi + p_hmi_H * 1; % unity prior for previous estimate bias
        else
            obj.p_hmi= obj.p_hmi + p_hmi_H * obj.p_H;
        end
    end
    
    % store integrity related data
    data.store_integrity_data(obj, counters)
end

% update the preceding horizon
update_preceding_horizon(obj, estimator)
end