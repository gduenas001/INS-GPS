
function compute_gamma_M_vector(obj, estimator)

if isempty(obj.gamma_M)
    obj.gamma_M= estimator.gamma_k;
    for i= 1:obj.M
        n_start= estimator.n_k + sum( obj.n_ph(1:i-1) ) + 1;
        n_end=   estimator.n_k + sum( obj.n_ph(1:i) );
        obj.gamma_M( n_start:n_end )= obj.gamma_ph{i};
    end
else % update gamma_M
    obj.gamma_M= [ estimator.gamma_k; obj.gamma_M( 1:sum(obj.n_ph(1:obj.M)) ) ];
end

end