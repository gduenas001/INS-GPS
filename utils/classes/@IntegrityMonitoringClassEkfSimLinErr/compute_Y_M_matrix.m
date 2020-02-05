
function compute_Y_M_matrix(obj, estimator)

% if it's the first epoch --> build the Y_M
if isempty(obj.Y_M)
    obj.Y_M= zeros( obj.n_M, obj.n_M );
    obj.Y_M( 1:estimator.n_k, 1:estimator.n_k )= estimator.Y_k;
    obj.Y_M_ub= zeros( obj.n_M, obj.n_M );
    obj.Y_M_ub( 1:estimator.n_k, 1:estimator.n_k )= estimator.Y_k_ub;
    for i= 1:obj.M
        n_start= estimator.n_k + sum( obj.n_ph(1:i-1) ) + 1;
        n_end=   estimator.n_k + sum( obj.n_ph(1:i) );
        obj.Y_M( n_start: n_end , n_start:n_end )= obj.Y_ph{i};
        obj.Y_M_ub( n_start: n_end , n_start:n_end )= obj.Y_ph_ub{i};
    end
else % update Y_M
    obj.Y_M= [ estimator.Y_k, zeros(estimator.n_k,sum(obj.n_ph(1:obj.M))) ;...
               zeros(sum(obj.n_ph(1:obj.M)),estimator.n_k), obj.Y_M(1:sum(obj.n_ph(1:obj.M)), 1:sum(obj.n_ph(1:obj.M)))];
    obj.Y_M_ub= [ estimator.Y_k_ub, zeros(estimator.n_k,sum(obj.n_ph(1:obj.M))) ;...
               zeros(sum(obj.n_ph(1:obj.M)),estimator.n_k), obj.Y_M_ub(1:sum(obj.n_ph(1:obj.M)), 1:sum(obj.n_ph(1:obj.M)))];
end
end
