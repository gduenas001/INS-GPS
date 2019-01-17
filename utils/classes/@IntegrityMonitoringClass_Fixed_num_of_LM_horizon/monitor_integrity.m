
function monitor_integrity(obj, estimator, counters, data, params)

% keep only the elements for the [x-y-theta]
obj.Phi_k= estimator.Phi_k^12; obj.Phi_k= obj.Phi_k( obj.ind_im, obj.ind_im ); %%%%%%%% CAREFUL
obj.H_k= estimator.H_k(:, obj.ind_im);
obj.L_k= estimator.L_k(obj.ind_im,:);


% monitor integrity if there are enough epochs
if counters.k_im > obj.M + 2 % need an extra two epoch to store Lpp (osama)
    
    % common parameters
    alpha= [-sin(estimator.XX(3)); cos(estimator.XX(3)); zeros(obj.m-2,1)];
    obj.sigma_hat= sqrt(alpha' * estimator.PX(obj.ind_im, obj.ind_im) * alpha);
    obj.n_M= sum( obj.n_ph ) + estimator.n_k;
    obj.T_d = sqrt( chi2inv( 1 - params.continuity_requirement , obj.n_M ) );
    obj.n_L_M= obj.n_M / params.m_F;
    obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    
    % Update the innovation vector covarience matrix for the new PH
    obj.compute_Y_M_matrix(estimator)
    
    % compute the A matrix for the preceding horizon
    obj.compute_A_M_matrix(estimator)
        
    % compute B_bar matrix
    obj.compute_B_bar_matrix(estimator)    
    
    % M matrix
    obj.M_M= obj.B_bar' / obj.Y_M * obj.B_bar;
    
    % set the threshold from the continuity req
    obj.detector_threshold= chi2inv(1 - obj.C_req, obj.n_M);
    
    obj.P_MA_M = [ obj.P_MA_k ; cell2mat(obj.P_MA_ph') ];
    
    % compute detector
    obj.q_M= sum(obj.q_ph) + estimator.q_k;
    
    % Loop over hypotheses in the PH (only 1 fault)
    n_H= obj.n_M / params.m_F; % one hypothesis per associated landmark in ph
    obj.p_hmi= 0;
    for i= 0:0%n_H
        % build extraction matrix 
        obj.compute_E_matrix(i, params.m_F)
        
        % Worst-case fault direction
        f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A_M' * alpha;
        f_M_dir= f_M_dir / norm(f_M_dir); % normalize
        
        % worst-case fault magnitude
        fx_hat_dir= alpha' * obj.A_M * f_M_dir;
        M_dir= f_M_dir' * obj.M_M * f_M_dir;
        
%         [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag)...
%             -((1-   normcdf(params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat)   +...
%             normcdf(-params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat))...
%             * ncx2cdf(obj.detector_threshold, params.m_F * obj.n_L_M, f_M_mag^2 * M_dir )),...
%             -10, 10);
        
        [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
            f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M),...
            -10, 10);
        
        
        p_hmi_H= -p_hmi_H; % make it a positive number
        
        % Add P(HMI | H) to the integrity risk
        if i == 0
            % previous-estimate fault hypothesis probability
            % scaled to make sure that the sum of all hypotheses' probabilities
            % equals to one (Total probability theorm)
            P_H = obj.p_prev_f_CA / ( obj.p_prev_f_CA + n_H*obj.p_UA + sum(obj.P_MA_M) ); 
            obj.p_hmi= obj.p_hmi + p_hmi_H * P_H;
        else
            % UA & MA fault hypothesis probability (for one landmark)
            % scaled to make sure that the sum of all hypotheses' probabilities
            % equals to one (Total probability theorm)
            P_H = ( obj.p_UA + obj.P_MA_M(i) ) / ( obj.p_prev_f_CA + n_H*obj.p_UA + sum(obj.P_MA_M) ); 
            obj.p_hmi= obj.p_hmi + p_hmi_H * P_H;
        end
    end
    % store integrity related data
    data.store_integrity_data(obj, counters, params)

elseif counters.k_im > 1 % if it's only 1 --> cannot compute Lpp_k
    obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
else
    obj.Lpp_k= 0;
end

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
update_preceding_horizon(obj, estimator)
end