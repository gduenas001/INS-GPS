
function compute_r_max(obj, params)

% probability of an association fault
P_F_M= obj.P_MA_M + params.P_UA;

for r= 0:length(P_F_M)
    if  (sum(P_F_M))^r  / factorial(r)  < params.I_H
        obj.n_max= r-1;
        break
    end
end

end
