
function monitor_integrity(obj, estimator, counters, data, params)

% keep only the elements for the [x-y-theta]
obj.Phi_k= estimator.Phi_k^12; obj.Phi_k= obj.Phi_k( obj.ind_im, obj.ind_im ); %%%%%%%% CAREFUL

% accounting for the case where there are no landmarks in the FoV at epoch k
if isempty(estimator.H_k) && isempty(estimator.L_k)
    obj.H_k= [];
    obj.L_k= [];
else
    obj.H_k= estimator.H_k(:, obj.ind_im);
    obj.L_k= estimator.L_k(obj.ind_im,:);
end

%add an extra epoch (initially) for matrices construction
if (sum( obj.n_ph ) + estimator.n_k >= params.min_num_LMs_to_monitor*params.m_F && obj.Extra_epoch_is_need== -1)
    obj.Extra_epoch_is_need= 0;
end

% monitor integrity if the number of LMs in the preceding horizon is more
% than a specifc threshold
if ( sum( obj.n_ph ) + estimator.n_k >= params.min_num_LMs_to_monitor*params.m_F && obj.Extra_epoch_is_need)
    
    % common parameters
    alpha= [-sin(estimator.XX(3)); cos(estimator.XX(3)); zeros(obj.m-2,1)];
    obj.sigma_hat= sqrt(alpha' * estimator.PX(obj.ind_im, obj.ind_im) * alpha);
    %obj.n_M= sum( obj.n_ph ) + estimator.n_k;
    %obj.n_L_M= obj.n_M / params.m_F;
    
    % Find the number of epochs needed to maintain a (specific) minimum number of
    % landmarks in the preceding horizon
    for i = 0 : size(obj.n_ph,1)
        if i == 0
            dummy_var = estimator.n_k;
        else
            dummy_var = dummy_var + obj.n_ph(i);
            if ( dummy_var >= params.min_num_LMs_to_monitor*params.m_F ) || ( i == size(obj.n_ph,1) )
                obj.n_M_for_LM = dummy_var;
                obj.n_L_M_for_LM = dummy_var/params.m_F;
                obj.M_for_LM = i;
                break;
            end
        end
    end
    
    obj.T_d = sqrt( chi2inv( 1 - params.continuity_requirement , obj.n_M_for_LM ) );
    
    % accounting for the case where there are no landmarks in the FoV at epoch k
    if (estimator.n_k == 0)
        obj.Lpp_k= obj.Phi_ph{1};
    else
        obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    end
    
    % accounting for the case where there are no landmarks in the FoV at
    % epoch k and the whole preceding horizon
    if (obj.n_M_for_LM == 0)
        obj.Y_M=[];
        obj.B_bar=[];
        obj.A_M=[];
        obj.q_M=0;
        obj.p_hmi= 1;
    else
        % Update the innovation vector covarience matrix for the new PH
        obj.compute_Y_M_matrix(estimator)

        % compute the A matrix for the preceding horizon
        obj.compute_A_M_matrix(estimator)

        % compute B_bar matrix
        obj.compute_B_bar_matrix(estimator)    

        % M matrix
        obj.M_M= obj.B_bar' / obj.Y_M * obj.B_bar;

        % set the threshold from the continuity req
        obj.detector_threshold= chi2inv(1 - obj.C_req, obj.n_M_for_LM);

        obj.P_MA_M = [ obj.P_MA_k ; cell2mat(obj.P_MA_ph(1:obj.M_for_LM)') ];

        % compute detector
        obj.q_M= sum(obj.q_ph(1:obj.M_for_LM)) + estimator.q_k;

        % Loop over hypotheses in the PH (only 1 fault)
        n_H= obj.n_M_for_LM / params.m_F; % one hypothesis per associated landmark in ph
        
        %initialization of p_hmi
        obj.p_hmi= 0;

        % (Single) sensor faults can't be monitored if the number of 
        % landmarks are less than or equal to two
        if n_H <= 2
            n_H = 0;
        end
        
        % variable to normalize P_H
        %norm_P_H= 0;
        
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
                f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M_for_LM),...
                0, 5);

            if abs(f_M_mag_out) > 4 && p_hmi_H > 1e-40 %  CAREFULLLLL
                error('optimization wrong')
            end
            p_hmi_H= -p_hmi_H; % make it a positive number

            % Add P(HMI | H) to the integrity risk
            if i == 0
                % previous-estimate fault hypothesis probability
                % scaled to make sure that the sum of all hypotheses' probabilities
                % equals to one (Total probability theorm)
%                 P_H = prod( 1-(obj.P_MA_M + params.p_UA) );
                P_H= 1;
%                 norm_P_H= norm_P_H + P_H;
                obj.p_hmi= obj.p_hmi + p_hmi_H * P_H;
            else
                % UA & MA fault hypothesis probability (for one landmark)
                % scaled to make sure that the sum of all hypotheses' probabilities
%                 % equals to one (Total probability theorm)
%                 P_H = 1; % initialization
%                 for j = 1 : size(obj.n_L_M_for_LM)
%                     if i == j
%                         P_H = P_H * (obj.P_MA_M(j) + params.p_UA);
%                         elsec
%                         P_H = P_H * (1 -(obj.P_MA_M(j) + params.p_UA));
%                     end
%                 end
%                 norm_P_H= norm_P_H + P_H;

%                 P_H= obj.P_MA_M(i) + params.p_UA;
                P_H= params.p_UA;
                obj.p_hmi= obj.p_hmi + p_hmi_H * P_H;
            end
        end
%         obj.p_hmi = ( (obj.p_hmi / norm_P_H));%*(1-params.p_UA*n_H)) + params.p_UA*n_H )*(1-params.I_MA) + params.I_MA;
    end
    % store integrity related data
    data.store_integrity_data(obj, counters, params)

elseif counters.k_im > 1 % if it's only 1 --> cannot compute Lpp_k
    obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    obj.M_for_LM = obj.M_for_LM +1;
    if obj.Extra_epoch_is_need == 0
        obj.Extra_epoch_is_need= 1;
    end
else
    obj.Lpp_k= 0;
    obj.M_for_LM = obj.M_for_LM +1;
    if obj.Extra_epoch_is_need == 0
        obj.Extra_epoch_is_need= 1;
    end
end

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
update_preceding_horizon(obj, estimator)
end