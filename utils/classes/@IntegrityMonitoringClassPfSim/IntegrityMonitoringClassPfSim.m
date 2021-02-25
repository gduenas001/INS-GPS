
classdef IntegrityMonitoringClassPfSim < handle
    properties (Constant)
        m= 3
        calculate_A_M_recursively = 0;
    end
    properties (SetAccess = immutable)
        C_req
    end
    properties
        
        p_hmi
        detector_threshold
        
        is_extra_epoch_needed= -1 % initialize as (-1), then a boolean
        ind_im= [1,2,9];

        
        % (maybe unnecessary)
        E
        B_bar
        
        % hypotheses
        inds_H % faulted indexes under H hypotheses
        P_H_0
        P_H
        T_d
        n_H
        n_max
        
        % for MA purposes
        mu_k
        kappa

        % current-time (k) only used when needed to extract elements
        sigma_hat = 0
        Phi_k
        H_k
        L_k
        Lpp_k
        P_MA_k
        P_MA_k_full
        
        % augmented (M) 
        M= 0  % size of the preceding horizon in epochs
        n_M   % num msmts in the preceding horizon (including k) -if FG ---> num abs msmts
        n_L_M=0 % num landmarks in the preceding horizon (including k)
        Phi_M
        q_M
        gamma_M
        Y_M
        A_M
        M_M
        P_MA_M
        P_F_M
        
        % preceding horizon saved (ph)
        Phi_ph
        q_ph
        gamma_ph
        A_ph
        L_ph
        Lpp_ph
        H_ph
        Y_ph
        P_MA_ph
        n_ph
        n_F_ph % number of features associated in the preceding horizon

        
        % Factor Graph variables
        m_M       % number of states to estimate
        n_total   % total number of msmts (prior + relative + abs)
        XX_ph
        D_bar_ph
        A
        Gamma_fg % information matrix
        M_fg
        PX_prior
        PX_M
        abs_msmt_ind
        faulted_LMs_indices
        Gamma_prior
        lidar_msmt_ind
        gps_msmt_ind
        n_gps_ph % number of gps msmt at each epoch in PH
        H_gps_ph
        H_lidar_ph
        n_M_gps
        A_reduced
        min_f_dir_vs_M_dir
        f_mag
        
        noncentral_dof= cell(10000,1)
        f_dir_sig2= cell(10000,1)
        M_dir= cell(10000,1)
        counter_H=0
        p_hmi_elapsed_time=0
        f_avg=0
        f_M_mag_out=0
        G_l_t=0
        
        M_k_update = 0
        N_k_update = 0
        N_r_2_update = 0
        M_k_predict = 0
        N_k_predict = 0
        N_r_2_predict = 0
        M_k_prior = 0
        N_k_prior = 0
        N_r_2_prior = 0
        q_k = 0
        n_L_k = 0
        alpha
        optimization_scale = 1e0
        Y_k
        beta_k
        eta_k_min
        %H_G_particles
        U_k
        Unique_particles_indices_prior
        eta_k_max
    end
    
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= IntegrityMonitoringClassPfSim(params, estimator)
            
            % if the preceding horizon is fixed in epochs --> set M
            %if params.SWITCH_FIXED_LM_SIZE_PH
            %    obj.M= 0;
            %else
            %    obj.M= params.M;
            %end
            
            % if it's a simulation --> change the indexes
            if params.SWITCH_SIM, obj.ind_im= 1:3; end
            
            % continuity requirement
            obj.C_req= params.continuity_requirement;
            % initialize the preceding horizon
            %obj.XX_ph=     cell(1, params.preceding_horizon_size+1);
            %obj.XX_ph{1}=  estimator.XX_updated;
            %obj.D_bar_ph=  cell(1, params.preceding_horizon_size);
            %obj.PX_prior=  estimator.PX_prior;
            %obj.Gamma_prior= estimator.Gamma_prior;
            %obj.n_gps_ph= zeros( params.preceding_horizon_size, 1 );
            %obj.H_gps_ph= cell( 1, params.preceding_horizon_size );
            %obj.H_lidar_ph= cell( 1, params.preceding_horizon_size );
            %obj.n_ph=     zeros( params.M, 1 );
            %obj.Phi_ph=   cell( 1, params.M + 1 ); % need an extra epoch here
            %obj.H_ph=     cell( 1, params.M );
            %obj.gamma_ph= cell(1, params.M);
            %obj.q_ph=     ones(params.M, 1) * (-1);
            %obj.L_ph=     cell(1, params.M);
            %obj.Lpp_ph=   cell(1, params.M + 1); % need an extra epoch here (osama)
            %obj.Y_ph=     cell(1, params.M);
            %obj.P_MA_ph=  cell(1, params.M);
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        %function neg_p_hmi= optimization_fn(obj, f_M_mag, fx_hat_dir, M_dir, sigma_hat, l, dof)
        %    neg_p_hmi= - ( (1 - normcdf(l , f_M_mag * fx_hat_dir, sigma_hat) +...
        %        normcdf(-l , f_M_mag * fx_hat_dir, sigma_hat))...
        %        * ncx2cdf(obj.T_d, dof, f_M_mag.^2 * M_dir ) );
        %end
        % ----------------------------------------------
        % ----------------------------------------------
        function neg_p_hmi= optimization_fn(obj, f_hat_sig_hat_f_h, params, estimator)
            f_prior_hat = f_hat_sig_hat_f_h(1:params.m)';
            
            %Cov_prior_hat = reshape(f_hat_sig_hat_f_h(params.m+1:params.m*(1+params.m) ),[params.m,params.m]);
            %Cov_prior_hat = [f_hat_sig_hat_f_h(params.m+1), f_hat_sig_hat_f_h(params.m+2), f_hat_sig_hat_f_h(params.m+3);
            %                 f_hat_sig_hat_f_h(params.m+2), f_hat_sig_hat_f_h(params.m+4), f_hat_sig_hat_f_h(params.m+5);
            %                 f_hat_sig_hat_f_h(params.m+3), f_hat_sig_hat_f_h(params.m+5), f_hat_sig_hat_f_h(params.m+6)];
            
            Cov_prior_hat = diag(f_hat_sig_hat_f_h(params.m+1:2*params.m));
                        
            if sum( eig(Cov_prior_hat) >= 0 ) < 3
                neg_p_hmi = 0;
                -eig(Cov_prior_hat)';
                return;
            end
                         
            if isempty(obj.E)
                f_k_h = zeros(estimator.n_k,1);
            else
                %E_f_k_h = f_hat_sig_hat_f_h(params.m*(1+params.m)+1:end)';
                %E_f_k_h = f_hat_sig_hat_f_h(params.m*(params.m+3)/2+1:end)';
                % % E_f_k_h = f_hat_sig_hat_f_h(2*params.m+1:end-1)';
                E_f_k_h = f_hat_sig_hat_f_h(2*params.m+1:end)';
                f_k_h = obj.E' * E_f_k_h;
            end
            
            % %eta_k_opt = f_hat_sig_hat_f_h(end);
            
            
            %f_k_pp = estimator.Phi_k * f_prior_hat + estimator.g_k_bar - estimator.XX_predict;
            %Cov_k_pp = (obj.N_r_2_predict/(obj.N_k_predict^2)) * estimator.Phi_k * Cov_prior_hat * estimator.Phi_k' + estimator.D_bar;
            
            %inv_Cov_k_pp = inv(Cov_k_pp);
            inv_V_k = inv(estimator.V_k);
            %M_k = 0.5*estimator.H_k'*inv_V_k*estimator.H_k + inv_Cov_k_pp;
            %M_k_inv = inv( 0.5*estimator.H_k'*inv_V_k*estimator.H_k + inv_Cov_k_pp );
            %D_k = 0.5*estimator.H_k*M_k_inv*(estimator.H_k'*inv_V_k) - eye(estimator.n_k);
            
            %E_E = inf * ones(params.m, obj.N_k_predict);
            %EXP_E = inf * ones(obj.N_k_predict,1);
            E_E = inf * ones(params.m, obj.N_k_prior);
            E_Cov_1 = inf * ones(params.m, obj.N_k_prior);
            EXP_E = inf * ones(obj.N_k_prior,1);
            EXP_Cov_1 = inf * ones(obj.N_k_prior,1);
            %E_E = inf * ones(params.m, obj.N_k_predict);
            %EXP_E = inf * ones(obj.N_k_predict,1);
            %gamma_k_i = inf * ones(params.m,obj.N_k_predict);
            %f_k_p_i = inf*ones(params.m,obj.N_k_prior);
            %Cov_k_p_i = inf*ones(params.m,obj.N_k_prior*params.m);
            %M_k_i = inf*ones(params.m,obj.N_k_prior*params.m);
            Sum_inv_M_k_i_EXP_E_i = zeros(params.m);
            Sum_inv_B_k_i_EXP_Cov_1_i = zeros(params.m);
            E_z_minus_Avg_h_x_i = f_k_h;
            Var_z_minus_Avg_h_x_i = estimator.V_k;
            %for i = 1 : obj.N_k_predict%obj.N_k_predict
            for i = 1 : obj.N_k_prior
                G_k_i = estimator.G_k_particles(:,params.m*(i-1)+1:params.m*i);
                F_k_i = estimator.F_k_particles(params.m*(i-1)+1:params.m*i,:);
                H_k_i = estimator.H_k_particles(:,params.m*(i-1)+1:params.m*i);
                
                %f_k_p_i = G_k_i*(estimator.XX_prior-estimator.XX_particles_prior(i,:)'+f_prior_hat);
                f_k_p_i = F_k_i*(estimator.odometry_k-estimator.Control_for_each_particle(:,i)) + G_k_i*(estimator.XX_prior-estimator.XX_particles_prior(i,:)'+f_prior_hat);
                Cov_k_p_i = G_k_i*Cov_prior_hat*G_k_i' + F_k_i*params.W_odometry_sim*F_k_i';
                inv_Cov_k_p_i = inv(Cov_k_p_i);
                M_k_i = 0.5*H_k_i'*inv_V_k*H_k_i + inv_Cov_k_p_i;
                inv_M_k_i = inv(M_k_i);
                C_k_i = inv_M_k_i* [ inv_Cov_k_p_i , -0.5*H_k_i'*inv_V_k ];
                M_k_i_cup = [ inv_Cov_k_p_i , zeros(params.m,estimator.n_k); zeros(estimator.n_k, params.m), 0.5*inv_V_k ] - C_k_i'*M_k_i*C_k_i;
                f_k_i_cup = [f_k_p_i; f_k_h];
                %E_E(:,i) = M_k_i\(Cov_k_p_i\f_k_p_i - 0.5*H_k_i'*inv_V_k*f_k_h);
                E_E(:,i) = C_k_i*f_k_i_cup;
                %EXP_E(i) = exp(-0.5*( (f_k_h'*(0.5*inv_V_k - 0.25*inv_V_k*(H_k_i/M_k_i)*H_k_i'*inv_V_k)*f_k_h) + (f_k_p_i' *(inv_Cov_k_p_i - (inv_Cov_k_p_i/M_k_i)*inv_Cov_k_p_i)* f_k_p_i) ))/sqrt(det(Cov_k_p_i*M_k_i));
                EXP_E(i) = exp(-0.5*f_k_i_cup'*M_k_i_cup*f_k_i_cup)/sqrt(det(Cov_k_p_i*M_k_i));
                %Sum_M_k_i_EXP_E_i = Sum_M_k_i_EXP_E_i + M_k_i*EXP_E(i);
                Sum_inv_M_k_i_EXP_E_i = Sum_inv_M_k_i_EXP_E_i + inv_M_k_i*EXP_E(i);
                
                E_z_minus_Avg_h_x_i = E_z_minus_Avg_h_x_i + (H_k_i*f_k_p_i/obj.N_k_predict);
                Var_z_minus_Avg_h_x_i = Var_z_minus_Avg_h_x_i + ( 1/(obj.N_k_predict^2) * H_k_i * Cov_k_p_i * H_k_i');
                
                B_k_i = (2/3) * H_k_i'*inv_V_k*H_k_i + inv_Cov_k_p_i;
                inv_B_k_i = inv(B_k_i);
                L_k_i = [inv_Cov_k_p_i, -(2/3)* H_k_i'*inv_V_k ];
                B_k_i_cup = [inv_Cov_k_p_i, zeros(params.m,estimator.n_k); zeros(estimator.n_k,params.m), (2/3)*inv_V_k] - L_k_i'*inv_B_k_i*L_k_i;
                E_Cov_1(:,i) = inv_B_k_i*L_k_i*f_k_i_cup;
                EXP_Cov_1(i) = exp(-0.5*f_k_i_cup'*B_k_i_cup*f_k_i_cup)/sqrt(det(Cov_k_p_i*B_k_i));
                Sum_inv_B_k_i_EXP_Cov_1_i = Sum_inv_B_k_i_EXP_Cov_1_i + inv_B_k_i*EXP_Cov_1(i);
                %gamma_k_i(:,i) = 0.5*(estimator.H_k'*inv_V_k)*(estimator.H_k*estimator.XX_predict - estimator.h_k + estimator.h_k_i(:, i) - f_k_h) + inv_Cov_k_pp*(estimator.XX_predict + f_k_pp);
                %E_E(:,i) = M_k_inv*gamma_k_i(:,i) - estimator.XX_particles_predict(i,:)'; 
                %E_E(:,i) = estimator.XX_predict - estimator.XX_particles_predict(i,:)' + M_k_inv * ( 0.5 * (estimator.H_k'/estimator.V_k) * ( estimator.h_k_i(:, i) - estimator.h_k - f_k_h ) + inv_Cov_k_pp*f_k_pp );
                %if (i == 1)
                %    scale = -0.5*( estimator.h_k_i(:, i)'*(-0.5*(estimator.V_k\D_k))*estimator.h_k_i(:, i) + (estimator.h_k_i(:, i)'/estimator.V_k)*( D_k*(f_k_h+estimator.h_k)-estimator.H_k*M_k_inv*(Cov_k_pp\f_k_pp)) );
                %end
                %EXP_E(i) = exp( -0.5*( estimator.h_k_i(:, i)'*(-0.5*(estimator.V_k\D_k))*estimator.h_k_i(:, i) + (estimator.h_k_i(:, i)'/estimator.V_k)*( D_k*(f_k_h+estimator.h_k)-estimator.H_k*M_k_inv*(Cov_k_pp\f_k_pp)) ) - scale );
                %EXP_E(i) = -0.5*( estimator.h_k_i(:, i)'*(-0.5*(estimator.V_k\D_k))*estimator.h_k_i(:, i) + (estimator.h_k_i(:, i)'/estimator.V_k)*( D_k*(f_k_h+estimator.h_k)-estimator.H_k*M_k_inv*inv_Cov_k_pp*f_k_pp) );
                %EXP_E(i) = -0.5*( (estimator.H_k*estimator.XX_predict - estimator.h_k + estimator.h_k_i(:, i) - f_k_h)'*0.5*inv_V_k*(estimator.H_k*estimator.XX_predict - estimator.h_k + estimator.h_k_i(:, i) - f_k_h) + (estimator.XX_predict+f_k_pp)'*inv_Cov_k_pp*(estimator.XX_predict+f_k_pp) - gamma_k_i(:,i)'*M_k_inv*gamma_k_i(:,i) );
            end
            
            %Var_z_minus_Avg_h_x_i = obj.U_k+estimator.V_k;
            % for i = 1 : obj.M_k_prior
            %     N_r_2_prior_i= ( sum( estimator.particles_indices_prior == obj.Unique_particles_indices_prior(i) ) )^2;
            %     ind= find(estimator.particles_indices_prior == obj.Unique_particles_indices_prior(i),1);
            %     H_G_i = estimator.H_k_particles(:,params.m*(ind-1)+1:params.m*ind)*estimator.G_k_particles(:,params.m*(ind-1)+1:params.m*ind);
            %     Var_z_minus_Avg_h_x_i = Var_z_minus_Avg_h_x_i + ( N_r_2_prior_i/(obj.N_k_prior^2) * H_G_i * Cov_prior_hat * H_G_i');
            % end
            
            %EXP_E = EXP_E - max(EXP_E);
            
            EXP_E = exp(EXP_E);
            
            %E_x_hat_k = (E_E*EXP_E)/sum(EXP_E); 
            %E_x_hat_k = (E_E*EXP_E)/(obj.eta_k_hat*(2*pi)^(0.5*estimator.n_k) * det(2*estimator.V_k)^(0.5)*det(Cov_k_pp*M_k)^(0.5)); 
            % %E_x_hat_k = E_E*EXP_E/((2*pi)^(0.5*estimator.n_k) * det(2*estimator.V_k)^(0.5));
            % %E_x_hat_k = E_x_hat_k/eta_k_opt;
            E_x_hat_k = E_E*EXP_E/sum(EXP_E);
            
            %Cov_k_hat = M_k_inv - (E_x_hat_k*E_x_hat_k') + ((E_E * diag(EXP_E) * E_E')/sum(EXP_E));
            %Cov_k_hat = M_k_inv + (((E_E-E_x_hat_k) * diag(EXP_E) * (E_E-E_x_hat_k)')/sum(EXP_E));
            %Cov_k_hat = ( M_k_inv*sum(EXP_E) + (E_E-E_x_hat_k) * diag(EXP_E) * (E_E-E_x_hat_k)' )/(obj.eta_k_hat*(2*pi)^(0.5*estimator.n_k) * det(2*estimator.V_k)^(0.5)*det(Cov_k_pp*M_k)^(0.5));
            % %Cov_k_hat = (Sum_M_k_i_EXP_E_i + (E_E-E_x_hat_k) * diag(EXP_E) * (E_E-E_x_hat_k)')/((2*pi)^(0.5*estimator.n_k) * det(2*estimator.V_k)^(0.5));
            % %Cov_k_hat = Cov_k_hat/obj.eta_k_max;
            Cov_k_hat = (Sum_inv_M_k_i_EXP_E_i + E_E * diag(EXP_E) * E_E' - E_x_hat_k*E_x_hat_k'*sum(EXP_E))/sum(EXP_E);
            
            Cov_j_o_k = Sum_inv_B_k_i_EXP_Cov_1_i + E_Cov_1 * diag(EXP_Cov_1) * E_Cov_1' - E_x_hat_k*E_x_hat_k'*sum(EXP_Cov_1);
            
            EXP_Cov_2_sum=0;
            E_EXP_Cov_2_sum=0;
            for i = 1 : obj.N_k_prior
                G_k_i = estimator.G_k_particles(:,params.m*(i-1)+1:params.m*i);
                F_k_i = estimator.F_k_particles(params.m*(i-1)+1:params.m*i,:);
                H_k_i = estimator.H_k_particles(:,params.m*(i-1)+1:params.m*i);
                h_k_i = estimator.h_k_i(:, i);
                Cov_k_p_i = G_k_i*Cov_prior_hat*G_k_i' + F_k_i*params.W_odometry_sim*F_k_i';
                inv_Cov_k_p_i = inv(Cov_k_p_i);
                f_k_p_i = F_k_i*(estimator.odometry_k-estimator.Control_for_each_particle(:,i)) + G_k_i*(estimator.XX_prior-estimator.XX_particles_prior(i,:)'+f_prior_hat);
                for r = 1 : obj.N_k_prior
                    if i == r
                        continue;
                    end
                    G_k_r = estimator.G_k_particles(:,params.m*(r-1)+1:params.m*r);
                    F_k_r = estimator.F_k_particles(params.m*(r-1)+1:params.m*r,:);
                    H_k_r = estimator.H_k_particles(:,params.m*(r-1)+1:params.m*r);
                    h_k_r = estimator.h_k_i(:, r);
                    Cov_k_p_r = G_k_r*Cov_prior_hat*G_k_r' + F_k_r*params.W_odometry_sim*F_k_r';
                    inv_Cov_k_p_r = inv(Cov_k_p_r);
                    f_k_p_r = F_k_r*(estimator.odometry_k-estimator.Control_for_each_particle(:,r)) + G_k_r*(estimator.XX_prior-estimator.XX_particles_prior(r,:)'+f_prior_hat);
                    A_k_r = (1/6)*H_k_r'*inv_V_k*H_k_r+inv_Cov_k_p_r;
                    inv_A_k_r = inv(A_k_r);
                    D_k_i_r = (1/6)*H_k_i'*inv_V_k*H_k_i-(1/36)*H_k_i'*inv_V_k*H_k_r*inv_A_k_r*H_k_r'*inv_V_k*H_k_i+inv_Cov_k_p_i;
                    inv_D_k_i_r = inv(D_k_i_r);
                    U_k_i_r = (1/6)*H_k_i'*inv_V_k*H_k_r*inv_A_k_r;
                    Q_k_r = eye(estimator.n_k) - (1/6)*H_k_r*inv_A_k_r*H_k_r'*inv_V_k;
                    O_k_i_r = [inv_Cov_k_p_i, -(1/6)*H_k_i'*inv_V_k*H_k_r*inv_A_k_r*inv_Cov_k_p_r, -(1/3)*H_k_i'*inv_V_k*Q_k_r];
                    R_k_r = inv_A_k_r * [zeros(params.m), inv_Cov_k_p_r, -(1/3)*H_k_r'*inv_V_k];
                    f_k_i_r_tilde = [ f_k_p_i ; f_k_p_r ; f_k_h ];
                    J_k_i_r = -(O_k_i_r'*inv_D_k_i_r)*O_k_i_r + [inv_Cov_k_p_i, zeros(params.m,params.m), zeros(params.m, estimator.n_k); zeros(params.m, params.m), inv_Cov_k_p_r, zeros(params.m, estimator.n_k); zeros(estimator.n_k, params.m), zeros(estimator.n_k, params.m), (2/3)*inv_V_k] - [zeros(params.m, params.m), inv_Cov_k_p_r, -(1/3)*H_k_r'*inv_V_k]'*inv_A_k_r*[zeros(params.m, params.m), inv_Cov_k_p_r, -(1/3)*H_k_r'*inv_V_k];
                    EXP_Cov_2= exp(-0.5*(0.5*(h_k_i-h_k_r))'*(2*inv_V_k)*(0.5*(h_k_i-h_k_r)))*exp(-0.5*(f_k_i_r_tilde)'*J_k_i_r*(f_k_i_r_tilde))/sqrt(det(Cov_k_p_i*D_k_i_r*Cov_k_p_r*A_k_r));
                    EXP_Cov_2_sum= EXP_Cov_2_sum + EXP_Cov_2;
                    E_EXP_Cov_2 = inv_D_k_i_r*(-U_k_i_r+O_k_i_r*f_k_i_r_tilde*f_k_i_r_tilde'*(R_k_r'-O_k_i_r'*inv_D_k_i_r*U_k_i_r)-D_k_i_r*E_x_hat_k*E_x_hat_k')*EXP_Cov_2;
                    E_EXP_Cov_2_sum = E_EXP_Cov_2_sum + E_EXP_Cov_2;
                end
            end
            
            Cov_j_o_k = (Cov_j_o_k + E_EXP_Cov_2_sum)/(sum(EXP_Cov_1) + EXP_Cov_2_sum);
            rho_k = (obj.alpha'*Cov_j_o_k*obj.alpha)/(obj.alpha'*Cov_k_hat*obj.alpha);
            %lambda_k_h = (obj.M_k_prior-params.m)/(2*estimator.n_k) * (E_z_minus_Avg_h_x_i)'/(Var_z_minus_Avg_h_x_i)*(E_z_minus_Avg_h_x_i);
            lambda_k_h = (obj.N_k_predict-params.m)/(2*estimator.n_k) * (E_z_minus_Avg_h_x_i)'/(Var_z_minus_Avg_h_x_i)*(E_z_minus_Avg_h_x_i);
            %mu_k_h = obj.alpha'*E_x_hat_k / sqrt( (obj.N_r_2_update/(obj.N_k_update^2)) * obj.alpha' * Cov_k_hat * obj.alpha );
            mu_k_h = obj.alpha'*E_x_hat_k / sqrt( (1/(obj.N_k_update)) * obj.alpha' * Cov_k_hat * obj.alpha );
            %-8.4497e-08 9
            %neg_p_hmi= - ( (1 - nctcdf(params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h) +...
            %    nctcdf(-params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h))...
            %    * ncfcdf(obj.T_d,estimator.n_k,obj.M_k_predict-params.m,lambda_k_h) )
            
            %if abs(mu_k_h) > 1000
            %    mu_k_h = sign(mu_k_h)*1000;
            %end
            
            if isreal(mu_k_h)
                %neg_p_hmi= - ( (nctcdf(params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h,'upper') +...
                %nctcdf(-params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h))...
                %* ncfcdf(obj.T_d,estimator.n_k,obj.M_k_prior-params.m,lambda_k_h) )
                %neg_p_hmi= - ( (nctcdf(params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h,'upper') +...
                %nctcdf(-params.alert_limit/sqrt((obj.N_r_2_update/(obj.N_k_update^2))*obj.alpha'*estimator.SX_update*obj.alpha),obj.M_k_update-1,mu_k_h))...
                %* ncfcdf(obj.T_d,estimator.n_k,obj.M_k_prior-params.m,lambda_k_h) )
                neg_p_hmi= - ( (nctcdf(params.alert_limit/sqrt((1/(obj.N_k_update))*obj.alpha'*estimator.SX_update*obj.alpha),obj.N_k_update-1,mu_k_h,'upper') +...
                nctcdf(-params.alert_limit/sqrt((1/(obj.N_k_update))*obj.alpha'*estimator.SX_update*obj.alpha),obj.N_k_update-1,mu_k_h))...
                * ncfcdf(obj.T_d,estimator.n_k,obj.N_k_predict-params.m,lambda_k_h) )
            
            else
                neg_p_hmi= 0
            end
            neg_p_hmi = obj.optimization_scale * neg_p_hmi;
            
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function compute_E_matrix(obj, i, m_F, estimator)
            if i == 0 % E matrix for only previous state faults
                obj.E= [];
            else % E matrix with a single LM fault
                obj.E= zeros( m_F*length(i) , estimator.n_k );
                for j= 1:length(i)
                    obj.E( m_F*(j-1) + 1 : m_F*(j) , m_F*(i(j)-1) + 1 : m_F*(i(j)) )= eye(m_F); % landmark i faulted
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        compute_hypotheses(obj, params)
        % ----------------------------------------------
        % ----------------------------------------------
        monitor_integrity(obj, estimator, counters, data,  params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_whiten_jacobian_A(obj, estimator, params)
        % ----------------------------------------------
        % ----------------------------------------------
        compute_required_epochs_for_min_LMs(obj, params, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        build_state_of_interest_extraction_matrix(obj, params, current_state)
        % ----------------------------------------------
        % ----------------------------------------------
        p_hmi_H= compute_p_hmi_H(obj, fault_ind, params, estimator)
        % ----------------------------------------------
        % ----------------------------------------------
        [c,ceq]= constriant_fn(obj, f_hat_sig_hat_f_h, params)
        % ----------------------------------------------
        % ----------------------------------------------
        %prob_of_MA(obj, estimator, params)
        % ----------------------------------------------
        % ----------------------------------------------
        %function update_preceding_horizon(obj, estimator)
        %    
        %        obj.Phi_ph=   {inf, estimator.Phi_k, obj.Phi_ph{2:obj.M}};
        %        obj.H_ph=     {estimator.H_k,   obj.H_ph{1:obj.M-1}};
        %        obj.n_ph=     [estimator.n_k;   obj.n_ph(1:obj.M-1)];
        %        obj.XX_ph=    {estimator.XX,    obj.XX_ph{1:obj.M}};
        %        obj.D_bar_ph= {inf, estimator.D_bar, obj.D_bar_ph{2:obj.M}};
        %        obj.P_MA_ph= {obj.P_MA_k,obj.P_MA_ph{1:obj.M-1}};
        %end 
    end
end




