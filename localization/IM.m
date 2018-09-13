function [P_HMI, H_M_cell, Y_M, Y_M_cell, A_k, L_M_cell, Lpp_M_cell, n_M_array]=...
    IM (Phi_M , H_M_cell, A_k , Y_M_cell, alpha, L_M_cell, Lpp_M_cell,epoch)

% PX : states prediction covarience matrix
% Phi_M : state tranision matrix over the horizon, including the current (concatenated)
% H_M : observation matrix over the horizon, excluding the current (concatenated)
% L_M : Kalman gain over the horizon, excluding the current (concatenated)
% L_pp_M : L_prime_prime over the horizon, excluding the current (concatenated)
% A_k : previous A_k for recursive computations
% Y_M : Innovation vector covarience matrix during the horizon

global PX PARAMS


% For a cleaner notation
m= PARAMS.m;
M= PARAMS.M;
m_F= PARAMS.m_F;
[lm,n_L]= field_of_view_landmarks();
nk= n_L*m_F;
% TODO
n_M_array= (-1) * ones(1,M+1);
n_M_array(1)= nk;
for i= 1:M
    n_M_array(i+1)= size(Y_M_cell{i},1);
end
n_M= sum(n_M_array);
nL_M= n_M / m_F; % number of lm in the PH
n_H=  nL_M + 1; % number of hypotheses (only one lm fault in PH)

% TODO this depends on the miss-association probability
P_H= PARAMS.P_H;

% Initializa variables for current time
Hk= zeros(nk, m); % Observation matrix at the current time step
h_k= zeros(nk, 1); % Expected measurement

% models for the current time
for t= 1:n_L
    idx= ((t-1)*m_F)+1:t*m_F;
    [h_t,H_t]= compute_lm_model(lm(:,t));
    Hk( idx,:)= H_t;
    h_k( idx,:)= h_t;
end

V= kron(eye(n_L),PARAMS.R); % current measurements covarience matrix
Y_k= Hk*PX*Hk' + V; % Current innovations covarience matrix



Lk= PX * Hk' / Y_k;
P_Hat= PX - Lk*Hk*PX;
Lk_pp= Phi_M(1:m,1:m) - Lk*Hk* Phi_M(1:m,1:m); % Kalman Gain prime-prime 

% Add cells
H_M_cell= [ {Hk}, H_M_cell];
L_M_cell= [ {Lk} ,L_M_cell];
Lpp_M_cell= [ {Lk_pp} ,Lpp_M_cell];
Y_M_cell= [ {Y_k} ,Y_M_cell];
% Y_M=  [ Y_k, zeros(nk,size(Y_M,1))  ;
%         zeros(size(Y_M,1),nk), Y_M ];

% Update the innovation vector covarience matrix for the new PH
Y_M= zeros(n_M,n_M);
Y_M(1:n_M_array(1) , 1:n_M_array(1))= Y_M_cell{1};
for i= 1:M
    n_start= sum( n_M_array(1:i) ) + 1;
    n_end= sum( n_M_array(1:i+1) );
    Y_M( n_start: n_end , n_start:n_end )= Y_M_cell{i+1};
%     Y_M(nk+1:end , nk+1:end)= Y_M(1:end-nk,1:end-nk);
%     Y_M(1:nk,1:nk)= Y_k;
end

A_k= Lk;
% A_k= inf* ones( m, n_M + m );
% A_k(: , 1:nk)= Lk;
for i= 1:M
    if i == 1
        Dummy_Variable= Lk_pp;
    else
        Dummy_Variable= Dummy_Variable * Lpp_M_cell{i};
    end
%     A_k(:, nk*i + 1 : nk*(i+1) )= Dummy_Variable * L_M_cell{i+1};
    A_k= [A_k , Dummy_Variable * L_M_cell{i+1}];
end
A_k= [A_k , Dummy_Variable * Lpp_M_cell{M+1}];
% A_k( :,n_M+1 : end )= Dummy_Variable * Lpp_M_cell{ M + 1 };


% Augmented B
B_bar= inf* ones( n_M , n_M+m );
A_prev= Lk_pp \ A_k( : , nk + 1:end );
B_bar(1:nk,:)= [eye(nk), -Hk*Phi_M(1:m,:)*A_prev];

% Recursive computation of B
for i= 1:M
    A_prev= Lpp_M_cell{i+1} \ A_prev(:, n_M_array(i+1)+1:end);
    B= [eye(size(H_M_cell{i+1},1)) , -H_M_cell{i+1} * Phi_M(m*i+1:m*(i+1),:) * A_prev];
    B_bar( sum(n_M_array(1:i))+1 : sum(n_M_array(1:i+1)) , 1 : sum(n_M_array(1:i)) )= 0;
    B_bar( sum(n_M_array(1:i))+1 : sum(n_M_array(1:i+1)) , sum(n_M_array(1:i))+1 : end)= B;

%     A_prev= Lpp_M_cell{i+1} \ A_prev(:, nk+1:end);
%     B= [eye(nk), -H_M_cell{i+1} * Phi_M(m*i+1:m*(i+1),:) * A_prev];
%     B_bar(i*nk+1 : (i+1)*nk , 1 : nk*i)= 0;
%     B_bar(i*nk+1 : (i+1)*nk , nk*i+1 : end)= B;
end
M_k= B_bar' / Y_M * B_bar;

% Using Cells
H_M_cell(end)= [];
L_M_cell(end)= [];
Lpp_M_cell(end)= [];
Y_M_cell(end)= [];
% Y_M(end-nk-1:end, :)= [];
% Y_M(:,end-nk-1:end)= [];

% Detector threshold including the PH
T_D= chi2inv(1 - PARAMS.C_REQ, n_M);


%% Loop over hypotheses in the PH (only 1 fault)
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

    [~, P_HMI_H]= fminbnd( @(f_M_mag)...
        -((1-cdf('Normal',PARAMS.alert_limit, f_M_mag * fx_hat_dir, sigma2_hat)+...
        cdf('Normal',-PARAMS.alert_limit,f_M_mag * fx_hat_dir, sigma2_hat))...
        * cdf('Noncentral Chi-square',T_D,m_F*nL_M, f_M_mag^2 * M_dir )), -10, 10);
    P_HMI_H= -P_HMI_H;
    
    % Add P(HMI | H) to the integrity risk
    P_HMI= P_HMI + P_HMI_H * P_H;
end





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

