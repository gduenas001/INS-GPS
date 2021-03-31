
function monitor_integrity(obj, estimator, counters, data, params)

% keep only the elements for the [x-y-theta]
obj.Phi_k= estimator.Phi_k;

% build L and H for the current time using only the pose indexes
if estimator.n_k == 0 % no landmarks in the FoV at epoch k
    obj.H_k= [];
    obj.L_k= [];
    obj.p_hmi = 1;
    return;
else % extract the indexes from the pose
    obj.H_k= estimator.H_k(:, params.ind_pose);
    obj.L_k= estimator.L_k(params.ind_pose, :);
end

% current horizon measurements
%obj.n_M= sum( obj.n_ph ) + estimator.n_k;
%obj.n_L_M= obj.n_M / params.m_F;

% the first time we have enough preceding horizon
%if obj.is_extra_epoch_needed == -1 && obj.n_L_M >= params.min_n_L_M && counters.k_im > 2
%    obj.is_extra_epoch_needed= true;
%end

% monitor integrity if the number of LMs in the preceding horizon is more than threshold
%if  ( params.SWITCH_FIXED_LM_SIZE_PH &&...
%    obj.n_L_M >= params.min_n_L_M &&...
%    obj.is_extra_epoch_needed == false ) ||...
%    ( ~params.SWITCH_FIXED_LM_SIZE_PH &&...
%    counters.k_im > obj.M + 2 )
    
%     % Modify preceding horizon to have enough landmarks
%     if params.SWITCH_FIXED_LM_SIZE_PH
%         obj.n_M= estimator.n_k;
%         for i= 1:length(obj.n_ph)
%             obj.n_M= obj.n_M + obj.n_ph(i);
%             % if the preceding horizon is long enough --> stop
%             if obj.n_M >= params.min_n_L_M * params.m_F, break, end
%         end
%         % set the variables
%         obj.n_L_M= obj.n_M / params.m_F;
%         obj.M= i;
%     end
    
    % If there are no landmarks in the FoV at k 
    %if estimator.n_k == 0
    %    obj.Lpp_k= obj.Phi_ph{1};
    %else
    %    obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
    %end
    
    % accounting for the case where there are no landmarks in the FoV at
    % epoch k and the whole preceding horizon
    %if obj.n_M == 0
    %    obj.Y_M=   [];
    %    obj.A_M=   [];
    %    obj.B_bar= [];
    %    obj.q_M= 0;
    %    obj.detector_threshold= 0;
    %    obj.p_hmi= 1;
    %else
        % Update the innovation vector covarience matrix for the new PH
        %obj.compute_Y_M_matrix(estimator)

        % compute the A matrix for the preceding horizon
        %obj.compute_A_M_matrix(estimator)

        % compute B_bar matrix
        %obj.compute_B_bar_matrix(estimator)    

        % M matrix
        %obj.M_M= obj.B_bar' / obj.Y_M * obj.B_bar;
        
        % compute detector
        %obj.q_M= sum(obj.q_ph(1:obj.M)) + estimator.q_k;
        
        % TODO: very inefficient --> do not transform from cell to matrix
        %obj.P_MA_M = [ obj.P_MA_k ; cell2mat(obj.P_MA_ph(1:obj.M)') ];
        obj.num_of_states_to_track = length(estimator.XX)-params.m_F*sum(estimator.association==-1);
        obj.D_k = [ estimator.H_k ; eye(obj.num_of_states_to_track) ];
        obj.Delta_k = [estimator.R_k, zeros(size(estimator.R_k,1),obj.num_of_states_to_track);zeros(obj.num_of_states_to_track,size(estimator.R_k,1)),estimator.PX_prediction];
        obj.S_k = (((obj.D_k'/obj.Delta_k)*obj.D_k)\obj.D_k')/obj.Delta_k;
        obj.M_k = obj.Delta_k\(eye(obj.num_of_states_to_track+estimator.n_k)-obj.D_k*obj.S_k);
        obj.n_L_k = estimator.n_L_k;
        obj.n_L_M = obj.n_L_k;
        obj.n_k = estimator.n_k;
        obj.q_M = estimator.q_k;
        
        % common parameters
        alpha= [-sin(estimator.XX(params.ind_yaw)); cos(estimator.XX(params.ind_yaw)); 0;zeros(obj.num_of_states_to_track-obj.m,1)];
        obj.sigma_hat= sqrt( alpha' * estimator.PX_update(1:obj.num_of_states_to_track,1:obj.num_of_states_to_track) * alpha );
    
        % detector threshold
        obj.T_d= chi2inv( 1 - params.continuity_requirement , estimator.n_k );
        
        % fault probability of each association in the preceding horizon
        obj.P_F_M= params.P_UA*ones(obj.n_L_k,1);%obj.P_MA_M + params.P_UA;
        
        %obj.p_hmi = 1;
        %return;
        
        % compute the hypotheses (n_H, n_max, inds_H)
        obj.compute_hypotheses(params)
        
        % initialization of p_hmi
        obj.p_hmi= 0;
        if obj.n_L_k - obj.n_max < 2 % need at least 5 msmts (3 landmarks) to monitor one landmark fault
            fprintf('Not enough redundancy: n_L_M = %d, n_max = %d\n', obj.n_L_k, obj.n_max)
            obj.p_hmi= 1;
            
        else % if we don't have enough landmarks --> P(HMI)= 1   
            obj.P_H= ones(obj.n_H, 1) * inf; % initializing P_H vector
            for i= 0:obj.n_H
                % build extraction matrix
                if i == 0
                    obj.compute_E_matrix(0, params.m_F);
                else
                    obj.compute_E_matrix(obj.inds_H{i}, params.m_F);
                end
                
%                 % compute N matrix
%                 N= obj.E * obj.M_M * obj.E';
%                 sqrt_inv_N= inv(sqrtm(N));
%                 
%                 % compute D matrix
%                 D= sqrt_inv_N * obj.E * obj.A_M' * alpha;
%                 
%                 % compute the relation between non-centrality paramters
%                 kappa= obj.sigma_hat^(-2) * norm(D)^2;
%                 
%                 % compute norm on MA non-centrality parameter
%                 lambda= ( sqrt(obj.q_M) + sqrt(  chi2inv( 1 - params.I_MA , obj.n_M ) ) )^2;
%                 
%                 % compute mu
%                 mu= lambda * kappa;
%                 
%                 % compute P(HMI | H)
%                 p_hmi_H= 1 - ncx2cdf( params.alert_limit^2 / obj.sigma_hat^2 , 1, mu );

                
                % Worst-case fault direction
                f_k_dir= obj.E' / (obj.E * obj.M_k * obj.E') * obj.E * obj.S_k' * alpha;
                f_k_dir= f_k_dir / norm(f_k_dir); % normalize
                
                % worst-case fault magnitude
                fx_hat_dir= alpha' * obj.S_k * f_k_dir;
                M_dir= f_k_dir' * obj.M_k * f_k_dir;
                
%                 % check if we should start evaluating f mag at zero
%                 if abs(obj.optimization_fn(...
%                         0, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M )) > 1e-10
%                     % worst-case fault magnitude
%                     f_mag_min= 0;
%                     f_mag_max= 5;
%                     f_mag_inc= 5;
%                     p_hmi_H_prev= -1;
%                     for k= 1:10
%                         [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                             f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M ),...
%                             f_mag_min, f_mag_max);
% 
%                         % make it a positive number
%                         p_hmi_H= -p_hmi_H;
% 
%                         % check if the new P(HMI|H) is smaller
%                         if k == 1 || p_hmi_H_prev < p_hmi_H
%                             p_hmi_H_prev= p_hmi_H;
%                             f_mag_min= f_mag_min + f_mag_inc;
%                             f_mag_max= f_mag_max + f_mag_inc;
%                         else
%                             p_hmi_H= p_hmi_H_prev;
%                             break
%                         end
%                     end
%                     % make a general optimization first
%                 else
%                     p_hmi_H_1= 0;
%                     p_hmi_H_2= 0;
%                     p_hmi_H_3= 0;
%                     p_hmi_H_4= 0;
%                     f_M_mag_out_1= -10;
%                     f_M_mag_out_2= -10;
%                     f_M_mag_out_3= -10;
%                     f_M_mag_out_4= -10;
% 
%                     [f_M_mag_out_1, p_hmi_H_1]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                         f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M ),...
%                         -10, 10);
%                     p_hmi_H_1= -p_hmi_H_1;
%                     if p_hmi_H_1 < 1e-10 || f_M_mag_out_1 > 8
%                         [f_M_mag_out_2, p_hmi_H_2]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                             f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M ),...
%                             -100, 100);
%                         p_hmi_H_2= -p_hmi_H_2;
%                         if p_hmi_H_2 < 1e-10 || f_M_mag_out_2 > 80
%                             [f_M_mag_out_3, p_hmi_H_3]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                                 f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M ),...
%                                 -1000, 1000);
%                             p_hmi_H_3= -p_hmi_H_3;
%                             if p_hmi_H_2 < 1e-10 || f_M_mag_out_3 > 800
%                                 [f_M_mag_out_4, p_hmi_H_4]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                                     f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_M ),...
%                                     -100000, 100000);
%                                 p_hmi_H_4= -p_hmi_H_4;
%                             end
%                         end
%                     end
%                     [p_hmi_H,ind]= max( [p_hmi_H_1, p_hmi_H_2, p_hmi_H_3, p_hmi_H_4] );
%                     if ind==1
%                         f_M_mag_out= abs(f_M_mag_out_1);
%                     elseif ind==2
%                         f_M_mag_out= abs(f_M_mag_out_2);
%                     elseif ind==3
%                         f_M_mag_out= abs(f_M_mag_out_3);
%                     else
%                         f_M_mag_out= abs(f_M_mag_out_4);
%                     end
%                 end

                % check if we should start evaluating f mag at zero
                if abs(obj.optimization_fn(...
                        0, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k )) > 1e-10
                    % worst-case fault magnitude
                    f_mag_min= 0;
                    f_mag_max= 5;
                    f_mag_inc= 5;
                    p_hmi_H_prev= -1;
                    for k= 1:10
                        [f_k_mag_out, p_hmi_H]= fminbnd( @(f_k_mag) obj.optimization_fn(...
                            f_k_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k ),...
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
                    % make a general optimization first
                else
                    p_hmi_H_1= 0;
                    p_hmi_H_2= 0;
                    p_hmi_H_3= 0;
                    p_hmi_H_4= 0;
                    f_k_mag_out_1= -10;
                    f_k_mag_out_2= -10;
                    f_k_mag_out_3= -10;
                    f_k_mag_out_4= -10;

                    [f_k_mag_out_1, p_hmi_H_1]= fminbnd( @(f_k_mag) obj.optimization_fn(...
                        f_k_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k ),...
                        -10, 10);
                    p_hmi_H_1= -p_hmi_H_1;
                    if p_hmi_H_1 < 1e-10 || f_k_mag_out_1 > 8
                        [f_k_mag_out_2, p_hmi_H_2]= fminbnd( @(f_k_mag) obj.optimization_fn(...
                            f_k_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k ),...
                            -100, 100);
                        p_hmi_H_2= -p_hmi_H_2;
                        if p_hmi_H_2 < 1e-10 || f_k_mag_out_2 > 80
                            [f_k_mag_out_3, p_hmi_H_3]= fminbnd( @(f_k_mag) obj.optimization_fn(...
                                f_k_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k ),...
                                -1000, 1000);
                            p_hmi_H_3= -p_hmi_H_3;
                            if p_hmi_H_2 < 1e-10 || f_k_mag_out_3 > 800
                                [f_k_mag_out_4, p_hmi_H_4]= fminbnd( @(f_k_mag) obj.optimization_fn(...
                                    f_k_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k ),...
                                    -100000, 100000);
                                p_hmi_H_4= -p_hmi_H_4;
                            end
                        end
                    end
                    p_hmi_H= max( [p_hmi_H_1, p_hmi_H_2, p_hmi_H_3, p_hmi_H_4] );
                end
                
%                 % worst-case fault magnitude
%                 f_mag_min= 0;
%                 f_mag_max= 5;
%                 f_mag_inc= 5;
%                 p_hmi_H_prev= -1;
%                 for k= 1:10
%                     [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                         f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k),...
%                         f_mag_min, f_mag_max);
%                     
%                     % make it a positive number
%                     p_hmi_H= -p_hmi_H;
%                     
%                     % check if the new P(HMI|H) is smaller
%                     if k == 1 || p_hmi_H_prev < p_hmi_H
%                         p_hmi_H_prev= p_hmi_H;
%                         f_mag_min= f_mag_min + f_mag_inc;
%                         f_mag_max= f_mag_max + f_mag_inc;
%                     else
%                         p_hmi_H= p_hmi_H_prev;
%                         break
%                     end
%                 end
                
                % Add P(HMI | H) to the integrity risk
                if i == 0
                    obj.P_H_0= prod( 1 - obj.P_F_M );
                    obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H_0;
                else
                    % unfaulted_inds= all( 1:obj.n_L_M ~= fault_inds(i,:)', 1 );
                    obj.P_H(i)= prod( obj.P_F_M( obj.inds_H{i} ) ); %...
                    % * prod( 1 - P_F_M(unfaulted_inds)  );
                    obj.p_hmi= obj.p_hmi + p_hmi_H * obj.P_H(i);
                end
            end
        end
    %end
    % store integrity related data
    data.store_integrity_data(obj, estimator, counters, params)

%elseif counters.k_im > 1 % if it's the first time --> cannot compute Lpp_k
    
%    if estimator.n_k == 0
%        obj.Lpp_k= obj.Phi_ph{1};
%    else
%        obj.Lpp_k= obj.Phi_ph{1} - obj.L_k * obj.H_k * obj.Phi_ph{1};
%    end
    
%    if params.SWITCH_FIXED_LM_SIZE_PH
%        obj.M = obj.M + 1;
%        if obj.is_extra_epoch_needed == true
%            obj.is_extra_epoch_needed= false;
%        end
%    end
%else % first time we get lidar msmts
%    obj.Lpp_k= 0;

%    if params.SWITCH_FIXED_LM_SIZE_PH
%        if obj.is_extra_epoch_needed == true
%            obj.is_extra_epoch_needed= false;
%        end
%    end
%end

% store time
data.im.time(counters.k_im)= counters.time_sim;

% update the preceding horizon
%update_preceding_horizon(obj, estimator, params)
end