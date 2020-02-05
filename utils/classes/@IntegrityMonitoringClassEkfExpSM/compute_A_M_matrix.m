function compute_A_M_matrix(obj,estimator)

% build matrix A_M in the first time
if isempty(obj.A_M) || ~obj.calculate_A_M_recursively 
    
    % allocate L_k and initialize
    obj.A_M= zeros(obj.m, obj.n_M + obj.m);
    obj.A_M(:,1:estimator.n_k)= obj.L_k;
    
    for i= 1:obj.M
        if i == 1
            Dummy_Variable= obj.Lpp_k;
        else
            Dummy_Variable= Dummy_Variable * obj.Lpp_ph{i-1};
        end
        
        % if no landmarks in the FoV at some time in the preceding horizon
        if obj.n_ph(i) > 0
            n_start= estimator.n_k + sum( obj.n_ph(1:i-1) ) + 1;
            n_end=   estimator.n_k + sum( obj.n_ph(1:i) );
            obj.A_M(:,n_start : n_end)= Dummy_Variable * obj.L_ph{i};
        end
    end
    
    % last entry with only PSIs
    obj.A_M(:, obj.n_M+1 : obj.n_M + obj.m) = Dummy_Variable * obj.Lpp_ph{obj.M};
    
% calculate matrix A_M recusively
else
    obj.L_k
    obj.A_M=[obj.L_k, obj.Lpp_k*obj.A_M];
    obj.A_M(:, obj.n_M +1 : end-obj.m) = [];
    obj.A_M(:, end-obj.m+1 : end) = obj.A_M(:, end-obj.m+1 : end)/obj.Lpp_ph{obj.M +1};
end

end