
function solve(obj, counters, params)

% number of associated landmarks at k
obj.n_L_k= length(obj.association);


% compute M
if params.SWITCH_FIXED_LM_SIZE_PH
       
    % initialize number of associations in the ph
    obj.n_L_M= obj.n_L_k;

    is_enough_landmarks= false;
    for M= 1:length(obj.n_L_k_ph)
        obj.n_L_M= obj.n_L_M + obj.n_L_k_ph(M);
        
        % if the preceding horizon is long enough --> stop
        if obj.n_L_M >= params.min_n_L_M 
            is_enough_landmarks= true;
            M= M + 1;
            break
        end
    end
    
    % check that there are enough associations
    if is_enough_landmarks && length(obj.x_ph) >= M && ~isempty(obj.x_ph{M})
        obj.M= M;
    else % increase ph
        obj.M= obj.M + 1;
        return
    end
    
else
    % number of associations in the ph
    obj.n_L_M= obj.n_L_k + sum( obj.n_L_k_ph(1:obj.M-1) );
    
    % check that there are enough epochs
    if counters.k_lidar <= obj.M, return, end

end

% total number of states to estimate
obj.m_M= (obj.M + 1) * params.m;

% total number of measurements (relativ + absolute + prior)
obj.n_total= obj.m_M + obj.n_L_M * params.m_F;

% create optimization function 
fun= @(x) obj.optimization_fn(x, params);

% from cells to vectors
x_star= obj.from_estimator_to_vector(params);

% saves the prior separately
obj.x_prior= obj.x_ph{obj.M};

% solve the problem
[residual, grad, A, b]= obj.optimization_fn(x_star, params);

[x_star, obj.q_d,~,~,~,hessian] = fminunc(fun, x_star, params.optimoptions);

% multiply by two so that it fits the non-central chi-squared
obj.q_d= obj.q_d * 2;

% compute detector threshold
obj.T_d= chi2inv( 1- params.continuity_requirement, obj.n_total - obj.m_M );

% debuggin points to check jacobian
% [residual, grad, A, b]= obj.optimization_fn_fg(x_star, params);
% dif= inv(A'*A) - inv(hessian);
% dif( abs(dif) < 5 )= 0;
% diag(dif)

% from a vector to cells 
from_vector_to_estimator(obj, x_star, params);

% store the prior, x_(k-M+1), as a future msmt
% hessian= A' * A;

obj.PX= inv( hessian );
obj.PX_prior= obj.PX( params.m + 1 : 2 * params.m , params.m + 1 : 2 * params.m );
obj.PX= obj.PX( end - params.m + 1 : end , end - params.m + 1 : end  );
% obj.Gamma_prior= inv( PX(params.m+1:2*params.m, params.m+1:2*params.m) );

end



