
function solve_fg(obj, counters, params)


% total number of states to estimate
obj.m_M= (params.M + 1) * params.m;

% number of associated landmarks at k
obj.n_L_k= length(obj.association);

% number of associations in the ph
obj.n_L_M= obj.n_L_k + numel( cell2mat(obj.association_ph(1:params.M-1)') );

% total number of measurements (relativ + absolute + prior)
obj.n_total= obj.m_M + obj.n_L_M * params.m_F;

% check that there are enough epochs
if counters.k_lidar <= params.M, return, end

% create optimization function 
fun= @(x) obj.optimization_fn_fg(x, params);

% from cells to vectors
x_star= obj.from_estimator_to_vector(params);

% saves the prior separately
obj.x_prior= obj.x_ph{params.M};

% solve the problem
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
from_vector_to_estimator(obj, x_star, params)

% store the prior, x_(k-M+1), as a future msmt
% hessian= A' * A;

obj.PX_prior= inv( hessian );
obj.PX_prior= obj.PX_prior( params.m + 1 : 2 * params.m , params.m + 1 : 2 * params.m );
% obj.Gamma_prior= inv( PX(params.m+1:2*params.m, params.m+1:2*params.m) );

end



