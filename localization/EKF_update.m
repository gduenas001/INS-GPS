
function [q_D, T_D, gamma_M]=...
    EKF_update(z, idf, step,gamma_M,Y_M,n_M_array)

global XX PX PARAMS hlm Hlm DATA

m_F= PARAMS.m_F;
M= PARAMS.M;

% Define some variables
n_L= DATA.numAssoc(step);
nk= n_L * m_F;

% If no lms are associated --> return!
if nk == 0
    q_D= 0;
    T_D= 0;
    return;
end

% Update detector threshold
T_D= chi2inv(1 - PARAMS.C_REQ, nk*(M+1));

% Remove non-associated msmts
z(:, idf == 0)= [];
idf( idf== 0) = [];

% make R a block diagonal with higher dimension
R= kron(eye(n_L), PARAMS.R);

% create the models for the association
h= zeros(nk,1);
H= zeros(nk,3);
for i= 1:n_L
    ind= i*m_F - 1;
    h(ind:ind+1)= hlm{idf(i)};
    H(ind:ind+1,:)= Hlm{idf(i)};
end

% Compute innovations
gamma= z(:) - h;
gamma(2:2:end)= pi_to_pi(gamma(2:2:end));
Y_k= H*PX*H' + R;

% Update the estimate
K= PX*H'/Y_k;
PX= PX - K*H*PX;
XX= XX + K*gamma;

% Update the sequence of innovations
gamma_M= gamma_M(1: sum(n_M_array( 2:end )));
% gamma_M( end - n_M_array(end) + 1: end )= [];
% gamma_M( end-n_M_array(PARAMS.M)+1:end )= [];
gamma_M= [gamma; gamma_M];

% Update the number of msmts in the PH
% n_M_array(2:end)= n_M_array(1:end-1);
% n_M_array(1)= nk;

% Detector
q_D= gamma_M' / Y_M * gamma_M;
