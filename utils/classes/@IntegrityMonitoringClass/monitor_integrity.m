
function monitor_integrity(obj, estimator, counters, data, params)

% keep only the elements for the [x-y-theta]
if params.SWITCH_SIM
    obj.Phi_k= estimator.Phi_k;
else
    % the state evolution matrix from one lidar msmt to the next one
    obj.Phi_k= estimator.Phi_k^12;  %%%%%%%% CAREFUL
    obj.Phi_k= obj.Phi_k( params.ind_pose, params.ind_pose ); 
end


% build L and H for the current time using only the pose indexes
if estimator.n_k == 0 % no landmarks in the FoV at epoch k
    obj.H_k= [];
    obj.L_k= [];
else % extract the indexes from the pose
    obj.H_k= estimator.H_k(:, params.ind_pose);
    obj.L_k= estimator.L_k(params.ind_pose, :);
end

% current horizon measurements
obj.n_M= sum( obj.n_ph ) + estimator.n_k;
obj.n_L_M= obj.n_M / params.m_F;

% the first time we have enough preceding horizon
if obj.is_extra_epoch_needed == -1 && obj.n_L_M >= params.min_n_L_M && counters.k_im > 1
    obj.is_extra_epoch_needed= true;
end

% monitor integrity if the number of LMs in the preceding horizon is more than threshold
if  ( params.SWITCH_FIXED_LM_SIZE_PH &&...
    obj.n_L_M >= params.min_n_L_M &&...
    obj.is_extra_epoch_needed == false ) ||...
    ( ~params.SWITCH_FIXED_LM_SIZE_PH &&...
    counters.k_im > obj.M + 2 )
    
    % Modify preceding horizon to have enough landmarks
    if params.SWITCH_FIXED_LM_SIZE_PH
        obj.n_M= estimator.n_k;
        for i= 1:length(obj.n_ph)
            obj.n_M= obj.n_M + obj.n_ph(i);
            % if the preceding horizon is long enough --> stop
            if obj.n_M >= params.min_n_L_M * params.m_F, break, end
        end
        % set the variables
        obj.n_L_M= obj.n_M / params.m_F;
        obj.M= i;
    end
    
    % common parameters
    alpha= [-sin(estimator.XX(params.ind_yaw)); cos(estimator.XX(params.ind_yaw)); 0];
%     alpha= [1;0;0];
%     alpha= [0;1;0];
    obj.sigma_hat= sqrt( alpha' * estimator.PX(params.ind_pose, params.ind_pose) * alpha );
    
    % detector threshold
    obj.T_d = sqrt( chi2inv( 1 - params.continuity_requirement , obj.n_M ) );
    
    % If there are no landmarks in the FoV at k 
    if estimator.n_k == 0
        obj.Lpp_k= obj.Phi_ph{1};
    else
        obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    end
    
    % accounting for the case where there are no landmarks in the FoV at
    % epoch k and the whole preceding horizon
    if obj.n_M == 0
        obj.Y_M=   [];
        obj.A_M=   [];
        obj.B_bar= [];
        obj.q_M= 0;
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
        obj.detector_threshold= chi2inv(1 - obj.C_req, obj.n_M);
        
        % TODO: very inefficient --> do not transform from cell to matrix
        obj.P_MA_M = [ obj.P_MA_k ; cell2mat(obj.P_MA_ph(1:obj.M)') ];

        % compute detector
        obj.q_M= sum(obj.q_ph(1:obj.M)) + estimator.q_k;

        % Loop over hypotheses in the PH (only 1 fault)
        obj.n_H= obj.n_M / params.m_F; % one hypothesis per associated landmark in ph
        
        % initialization of p_hmi
        obj.p_hmi= 0;

        % need at least 5 msmts (3 landmarks) to monitor one landmark fault
        if obj.n_M < 5
            obj.p_hmi= 1;
            
        else % if we don't have enough landmarks --> P(HMI)= 1
            % variable to normalize P_H
            %norm_P_H= 0;
            
            for i= 0:1%obj.n_H
                % build extraction matrix
                obj.compute_E_matrix(i, params.m_F)
                
                % Worst-case fault direction
                f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A_M' * alpha;
                f_M_dir= f_M_dir / norm(f_M_dir); % normalize
                
                % worst-case fault magnitude
                fx_hat_dir= alpha' * obj.A_M * f_M_dir;
                M_dir= f_M_dir' * obj.M_M * f_M_dir;
                
                % worst-case fault magnitude
                f_mag_min= 0;
                f_mag_max= 5;
                f_mag_inc= 5;
                p_hmi_H_prev= -1;
                for j= 1:10
                    [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
                        f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M),...
                        f_mag_min, f_mag_max);
                    
                    % make it a positive number
                    p_hmi_H= -p_hmi_H;    
                    
                    % check if the new P(HMI|H) is smaller
                    if j == 1 || p_hmi_H_prev < p_hmi_H 
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
                    %                 for j = 1 : size(obj.n_L_M)
                    %                     if i == j
                    %                         P_H = P_H * (obj.P_MA_M(j) + params.p_UA);
                    %                         elsec
                    %                         P_H = P_H * (1 -(obj.P_MA_M(j) + params.p_UA));
                    %                     end
                    %                 end
                    %                 norm_P_H= norm_P_H + P_H;
                    
                    %                 P_H= obj.P_MA_M(i) + params.p_UA;
                    P_H= params.P_UA;
                    obj.p_hmi= obj.p_hmi + p_hmi_H * P_H;
                end
            end
            %         obj.p_hmi = ( (obj.p_hmi / norm_P_H));%*(1-params.p_UA*n_H)) + params.p_UA*n_H )*(1-params.I_MA) + params.I_MA;
        end
    end
    % store integrity related data
    data.store_integrity_data(obj, counters, params)

elseif counters.k_im > 1 % if it's the first time --> cannot compute Lpp_k
    
    if estimator.n_k == 0
        obj.Lpp_k= obj.Phi_ph{1};
    else
        obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    end
    
    if params.SWITCH_FIXED_LM_SIZE_PH
        obj.M = obj.M + 1;
        if obj.is_extra_epoch_needed == true
            obj.is_extra_epoch_needed= false;
        end
    end
else % first time we get lidar msmts
    obj.Lpp_k= 0;

    if params.SWITCH_FIXED_LM_SIZE_PH
        if obj.is_extra_epoch_needed == true
            obj.is_extra_epoch_needed= false;
        end
    end
end

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
update_preceding_horizon(obj, estimator, params)
end