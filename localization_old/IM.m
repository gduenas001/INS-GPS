
% PX : states prediction covarience matrix
% Phi_M : state tranision matrix over the horizon, including the current (concatenated)
% H_M : observation matrix over the horizon, excluding the current (concatenated)
% L_M : Kalman gain over the horizon, excluding the current (concatenated)
% L_pp_M : L_prime_prime over the horizon, excluding the current (concatenated)


clear; close all; 
load('STORE_M2_range25.mat');

% parameters
m= 3;
M= 2;
m_F= 2;
P_H= 1e-3;
C_REQ= 1e-5;
alert_limit= 1;

num_epochs= length(STORE.Phi_M);
% num_epochs= 200;

for k= 1:num_epochs
    
    % percentage completed
    disp(k)
    
    Phi_M_cell= STORE.Phi_M{k};
    gamma_M_cell= STORE.gamma_M{k};
    Y_M_cell= STORE.Y_M{k};
    L_M_cell= STORE.L_M{k};
    Lpp_M_cell= STORE.Lpp_M{k};
    H_M_cell= STORE.H_M{k};
    R_lidar= STORE.R_lidar;
    PX_bar= STORE.P_bar{k};
    XX_bar= STORE.XX{k};
    
    
    % for the current time k
    gamma_k= gamma_M_cell{1};
    nk= length(gamma_k);
    n_L= nk / m_F;
    H_k= H_M_cell{1};
    Y_k= Y_M_cell{1};
    L_k= L_M_cell{1};
    Lpp_k= Lpp_M_cell{1};
    
    % alpha
    alpha= [-sin(XX_bar(3));cos(XX_bar(3));0];
    
    % number of msmts in the PH
    n_M_array= (-1) * ones(1,M+1);
    n_M_array(1)= nk;
    for i= 1:M
        n_M_array(i+1)= size(Y_M_cell{i+1},1);
    end
    n_M= sum(n_M_array);
    nL_M= n_M / m_F; % number of lm in the PH
    n_H=  nL_M + 1; % number of hypotheses (only one lm fault in PH)
    
    P_Hat= PX_bar - L_k*H_k*PX_bar;
    
    % Update the innovation vector covarience matrix for the new PH
    Y_M= zeros(n_M,n_M);
    Y_M(1:n_M_array(1) , 1:n_M_array(1))= Y_M_cell{1};
    for i= 1:M
        n_start= sum( n_M_array(1:i) ) + 1;
        n_end= sum( n_M_array(1:i+1) );
        Y_M( n_start: n_end , n_start:n_end )= Y_M_cell{i+1};
    end
    
    A_k= L_k;
    for i= 1:M
        if i == 1
            Dummy_Variable= Lpp_k;
        else
            Dummy_Variable= Dummy_Variable * Lpp_M_cell{i};
        end
        A_k= [A_k , Dummy_Variable * L_M_cell{i+1}];
    end
    A_k= [A_k , Dummy_Variable * Lpp_M_cell{M+1}];
    
    
    % Augmented B
    B_bar= inf* ones( n_M , n_M+m );
    A_prev= Lpp_k \ A_k( : , nk + 1:end );
    B_bar(1:nk,:)= [eye(nk), -H_k*Phi_M_cell{2}*A_prev];
    
    % Recursive computation of B
    for i= 1:M
        A_prev= Lpp_M_cell{i+1} \ A_prev(:, n_M_array(i+1)+1:end);
        %         B= [eye(size(H_M_cell{i+1},1)) , -H_M_cell{i+1} * Phi_M (m*i+1:m*(i+1),:) * A_prev];
        B= [eye(size(H_M_cell{i+1},1)) , -H_M_cell{i+1} * Phi_M_cell{i+1} * A_prev];
        B_bar( sum(n_M_array(1:i))+1 : sum(n_M_array(1:i+1)) , 1 : sum(n_M_array(1:i)) )= 0;
        B_bar( sum(n_M_array(1:i))+1 : sum(n_M_array(1:i+1)) , sum(n_M_array(1:i))+1 : end)= B;
    end
    M_k= B_bar' / Y_M * B_bar;
        
    
    % Detector threshold including the PH
    T_D= chi2inv(1 - C_REQ, n_M);
    T_D_save(k)= T_D;
    
    % Loop over hypotheses in the PH (only 1 fault)
    P_HMI= 0;
    for i= 1:n_H
        
        if i == 1 % E matrix for only previous state faults
            E= zeros( m, n_M+m );
            E(:, end-m+1:end)= eye(m);
        else % E matrix with faults in the PH
            E= zeros( m + m_F , n_M + m );
            E( end-m+1 : end , end-m+1:end )= eye(m); % previous bias
            E( 1:m_F , (i-2)*m_F+1 : (i-1)*m_F )= eye(m_F); % landmark i faulted
        end
        
        % Worst-case fault direction
        f_M_dir= E' / (E*M_k*E') * E * A_k' * alpha;
        f_M_dir= f_M_dir / norm(f_M_dir);
        
        % worst-case fault magnitude
        sigma2_hat= alpha'*P_Hat*alpha;
        fx_hat_dir= alpha' * A_k * f_M_dir;
        M_dir= f_M_dir' * M_k * f_M_dir;
        
        [f_M_mag_out, P_HMI_H]= fminbnd( @(f_M_mag)...
            -((1-   normcdf(alert_limit, f_M_mag * fx_hat_dir, sqrt(sigma2_hat))   +...
            normcdf(-alert_limit,f_M_mag * fx_hat_dir, sqrt(sigma2_hat)))...
            * ncx2cdf(T_D, m_F*nL_M, f_M_mag^2 * M_dir )), -10, 10);
%             -((1-cdf('Normal', alert_limit, f_M_mag * fx_hat_dir, sigma2_hat)+...
%             cdf('Normal',-alert_limit,f_M_mag * fx_hat_dir, sigma2_hat))...
%             * cdf('Noncentral Chi-square',T_D,m_F*nL_M, f_M_mag^2 * M_dir )), -10, 10);
        
        P_HMI_H= -P_HMI_H;
        
        % Add P(HMI | H) to the integrity risk
        if i == 1
            P_HMI= P_HMI + P_HMI_H * 1;
        else
            P_HMI= P_HMI + P_HMI_H * P_H;
        end
    end
    DATA.P_HMI_save(k)= P_HMI;
    DATA.P_HMI_H0_save(k)= 2*normcdf(-alert_limit, 0, sqrt(sigma2_hat));
    
end % end of loop through STORE



% plots
figure; hold on; grid on;
plot(1:num_epochs, DATA.P_HMI_save, 'g-', 'linewidth', 2)
plot(1:num_epochs, DATA.P_HMI_H0_save, 'b-', 'linewidth', 2)
set(gca,'Yscale','log');


% figure; hold on; grid on;
% plot(1:num_epochs, [STORE.q_D{1:num_epochs}], 'r-', 'linewidth', 2)
% plot(1:num_epochs, T_D_save, 'b-', 'linewidth', 2)


% P_HMI_save
% P_HMI_H0_save











%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [h,H]= compute_lm_model(lm)

global XX

dx= lm(1) - XX(1);
dy= lm(2) - XX(2);
d2= dx^2 + dy^2;
d= sqrt(d2);

% calculate h
h= [d;
    pi_to_pi( atan2(dy,dx) - XX(3) )];

% calculate H
H = [-dx/d, -dy/d,  0;
    dy/d2, -dx/d2, -1];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [lm, nL]= field_of_view_landmarks ()

global XX PX LM PARAMS

% % calculate how much we need to include in the EFOV
%lambda_FV= sqrt( max(eig(Px(1:2,1:2))) );
%EFV= -sqrt(2) * lambda_FV * norminv(PARAMS.I_FOV/2,0,1);
% EFOV= sqrt(2) * lambda_FOV * sqrt( chi2inv(1 - PARAMS.I_FOV,1) ); % same as previous

% Get all visible landmarks, assuming no mis-extractions here
idf= get_visible_landmarks(XX,PARAMS.maxRange, 0);

lm= LM(:,idf);
nL= length(idf);

end