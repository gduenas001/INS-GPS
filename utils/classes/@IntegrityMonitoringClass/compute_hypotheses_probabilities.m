
function compute_hypotheses_probabilities(obj, params)

% probability of an association fault
P_F_M= obj.P_MA_M + params.P_UA;

% null hypothesis probability
obj.P_H_0= prod(1 - P_F_M);

% probability of faulted hypotheses
obj.P_H= ones(obj.n_L_M,1) * inf;
for i= 1:obj.n_L_M
    obj.P_H(i)= obj.P_H_0 * P_F_M(i) / (1 - P_F_M(i));
end


end
