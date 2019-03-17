
function compute_hypotheses_probabilities(obj, params)

% probability of an association fault
P_F_M= obj.P_MA_M + params.P_UA;

% probabilities cannot be more than one
P_F_M(P_F_M > 1)= 1;

% null hypothesis probability
obj.P_H_0= prod(1 - P_F_M);

% initialize landmark indexes
lm_inds= 1:obj.n_L_M;
if params.SWITCH_ONE_LANDMARK_FAULT % only one landmark may fail simultaneously
    obj.P_H= ones(obj.n_L_M, 1) * inf;
    for i= 1:obj.n_L_M
        unfaulted_inds= lm_inds ~= i;
        obj.P_H(i)= P_F_M(i) * prod(1 - P_F_M(unfaulted_inds));
    end
    
else % many simultaneous landmarks failing
    % maximum number of simultaneous faults
    n_max= 2;
    
    
end







% 
% % probability of faulted hypotheses
% obj.P_H= ones(obj.n_L_M,1) * inf;
% for i= 1:obj.n_L_M
%     obj.P_H(i)= obj.P_H_0 * P_F_M(i) / (1 - P_F_M(i));
% end


end
