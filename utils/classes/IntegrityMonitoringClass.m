
classdef IntegrityMonitoringClass < handle
    properties (Constant)
        m= 3
        p_H= 1e-3
        ind_im= [1,2,9];
    end
    properties (SetAccess = immutable)
        M % size of the preceding horizon in epochs
        C_req
    end
    properties
        
        p_hmi
        detector_threshold
        
        n_ph
        n_F_ph % number of features associated in the preceding horizon
        

        XX
        PX
        n_k
        gamma_k
        Phi_k
        H_k
        L_k
        Y_k
        Lpp_k
        q_k
        
        % augmented (M) 
        n_M % num msmts in the preceding horizon (including k)
        n_L_M % num landmarks in the preceding horizon (including k)
        Phi_M
        q_M
        gamma_M
        Y_M
        A_M
        M_M
        
        % preceding horizon saved (ph)
        Phi_ph
        q_ph
        gamma_ph
        A_ph
        L_ph
        Lpp_ph
        H_ph
        Y_ph
        
    end
    
    
    methods
        function obj= IntegrityMonitoringClass(params)
            obj.M= params.preceding_horizon_size;
            
            obj.n_ph=     ones(obj.M,1) * (-1);
            obj.Phi_ph=   cell(1, obj.M + 1); % need an extra epoch here
            obj.q_ph=   cell(1, obj.M);
            obj.gamma_ph= cell(1, obj.M);
            obj.L_ph=     cell(1, obj.M);
            obj.Lpp_ph=   cell(1, obj.M);
            obj.H_ph=     cell(1, obj.M);
            obj.Y_ph=     cell(1, obj.M);
            
            obj.C_req= params.continuity_requirement;
        end
        
        function monitor_integrity(obj, estimator, counters, data, params)
            
            % keep only the elements for the x-y-theta
            obj.XX= estimator.XX(obj.ind_im);
            obj.PX= estimator.PX(obj.ind_im, obj.ind_im);
            obj.n_k= estimator.n_k;
            obj.gamma_k= estimator.gamma_k;
            obj.Phi_k= estimator.Phi_k^12; obj.Phi_k= obj.Phi_k( obj.ind_im, obj.ind_im ); %%%%%%%% CAREFUL
            obj.H_k= estimator.H_k(:, obj.ind_im);
            obj.L_k= estimator.L_k(obj.ind_im,:);
            obj.Y_k= estimator.Y_k;
            
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
                obj.Y_M= zeros( obj.n_M, obj.n_M );
                obj.Y_M( 1:estimator.n_k, 1:estimator.n_k )= estimator.Y_k;
                for i= 1:obj.M
                    n_start= estimator.n_k + sum( obj.n_ph(1:i-1) ) + 1;
                    n_end=   estimator.n_k + sum( obj.n_ph(1:i) );
                    obj.Y_M( n_start: n_end , n_start:n_end )= obj.Y_ph{i};
                end
                
                % create matrix A_M at k
                obj.A_M= obj.L_k;
                for i= 1:obj.M
                    if i == 1
                        Dummy_Variable= obj.Lpp_k;
                    else
                        Dummy_Variable= Dummy_Variable * obj.Lpp_ph{i-1};
                    end
                    obj.A_M= [obj.A_M , Dummy_Variable * obj.L_ph{i}];
                end
                obj.A_M= [ obj.A_M , Dummy_Variable * obj.Lpp_ph{obj.M} ];
                
                % Augmented B
                B_bar= inf* ones( obj.n_M , obj.n_M + obj.m );
                A_prev= obj.Lpp_k \ obj.A_M( : , estimator.n_k + 1:end );
                B_bar(1:estimator.n_k , :)=...
                    [ eye(estimator.n_k), -obj.H_k * obj.Phi_ph{1} * A_prev ];
                B_ind_row_start= estimator.n_k + 1;
                B_ind_col_end= estimator.n_k;
                
                % Recursive computation of B
                for i= 1:obj.M
                    A_prev= obj.Lpp_ph{i} \ A_prev(:, obj.n_ph(i)+1:end);
                    B= [eye( obj.n_ph(i) ) , -obj.H_ph{i} * obj.Phi_ph{i+1} * A_prev];
                    
                    B_ind_row_end= B_ind_row_start + obj.n_ph(i) - 1;
                                        
                    B_bar(B_ind_row_start:B_ind_row_end, 1:B_ind_col_end)= 0;
                    B_bar(B_ind_row_start:B_ind_row_end, B_ind_col_end+1:end)= B;
                    
                    % increase row index for next element B
                    B_ind_row_start= B_ind_row_start + obj.n_ph(i);
                    B_ind_col_end= B_ind_col_end + obj.n_ph(i);
                end
                obj.M_M= B_bar' / obj.Y_M * B_bar;
                
                % set the threshold from the continuity req
                obj.detector_threshold= chi2inv(1 - obj.C_req, obj.n_M);
                
                % compute detector
                obj.gamma_M= [ estimator.gamma_k ; cell2mat(obj.gamma_ph') ];
                obj.q_k= obj.gamma_M' / obj.Y_M * obj.gamma_M;
                
                % Loop over hypotheses in the PH (only 1 fault)
                obj.p_hmi= 0;
                
                n_H= estimator.n_k / params.m_F; % one hypothesis per associated landmark in ph
                for i= 1:n_H
                    
                    if i == 1 % E matrix for only previous state faults
                        E= zeros( obj.m, obj.n_M + obj.m );
                        E(:, end-obj.m + 1:end)= eye(obj.m);
                    else % E matrix with faults in the PH
                        E= zeros( obj.m + params.m_F , obj.n_M + obj.m );
                        E( end-obj.m+1 : end , end-obj.m+1:end )= eye(obj.m); % previous bias
                        E( 1:params.m_F , (i-2)*params.m_F+1 : (i-1)*params.m_F )= eye(params.m_F); % landmark i faulted
                    end
                    
                    % Worst-case fault direction
                    f_M_dir= E' / (E * obj.M_M * E') * E * obj.A_M' * alpha;
                    f_M_dir= f_M_dir / norm(f_M_dir); % normalize
                    
                    % worst-case fault magnitude
                    sigma2_hat= alpha' * obj.PX * alpha;
                    fx_hat_dir= alpha' * obj.A_M * f_M_dir;
                    M_dir= f_M_dir' * obj.M_M * f_M_dir;
                    
                    [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag)...
                        -((1-   normcdf(params.alert_limit, f_M_mag * fx_hat_dir, sqrt(sigma2_hat))   +...
                        normcdf(-params.alert_limit, f_M_mag * fx_hat_dir, sqrt(sigma2_hat)))...
                        * ncx2cdf(obj.detector_threshold, params.m_F * obj.n_L_M, f_M_mag^2 * M_dir )),...
                        -10, 10);
                    
                    p_hmi_H= -p_hmi_H; % make it a positive number
                    
                    % Add P(HMI | H) to the integrity risk
                    if i == 1
                        obj.p_hmi= obj.p_hmi + p_hmi_H * 1; % unity prior for previous estimate bias
                    else
                        obj.p_hmi= obj.p_hmi + p_hmi_H * obj.p_H;
                    end    
                end
                
                % store integrity related data
                data.store_integrity_data(obj, counters)                
            end
           
            % update the preceding horizon
            obj.n_ph=     [obj.n_k; obj.n_ph(1:end-1)];
            obj.gamma_ph= [obj.gamma_k, obj.gamma_ph(1:end-1)];
            obj.Phi_ph=   [obj.Phi_k^12, obj.Phi_ph(1:end-1)]; %%%%%%%% CAREFUL
            obj.H_ph=     [obj.H_k, obj.H_ph(1:end-1)];
            obj.L_ph=     [obj.L_k, obj.L_ph(1:end-1)];
            obj.Lpp_ph=   [obj.Lpp_k, obj.Lpp_ph(1:end-1)];
            obj.Y_ph=     [obj.Y_k, obj.Y_ph(1:end-1)];
        end
    end
end




