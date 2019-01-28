
function compute_hypotheses(obj, params)

% probability of "r" or more simultaneous faults
for r= 1:length(obj.P_F_M)
    if  sum(obj.P_F_M)^r  / factorial(r)  < params.I_H
        obj.n_max= r-1;
        break
    end
end

if obj.n_max > 2
    fprintf('n_max: %d\n', obj.n_max);
    obj.n_max= 2;
end

% compute number of hypotheses
obj.n_H= 0;
obj.inds_H= cell(200,1);
start_ind= 1;
for num_faults= 1:obj.n_max
    new_H= nchoosek(obj.n_L_M, num_faults);
    obj.n_H= obj.n_H + new_H;
    obj.inds_H( start_ind:start_ind+new_H - 1, 1 )=...
        num2cell( nchoosek(1:obj.n_L_M, num_faults), 2 );
    start_ind= start_ind + new_H;
end
obj.inds_H(start_ind:end)= [];



end
