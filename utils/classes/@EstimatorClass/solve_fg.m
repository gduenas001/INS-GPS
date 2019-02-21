
function solve_fg(obj, counters, params)


% total number of states to estimate
obj.m_M= (params.M + 1) * params.m;

% total number of measurements
obj.n_total= length(obj.z_fg);

% check that there are enough epochs
if counters.k_lidar <= params.M, return, end

% create optimization function 
fun= @(x) obj.optimization_fn_fg(x, params);

% from cells to vectors
x_star= obj.from_estimator_to_vector(params);

% solve the problem
[x_star,~,~,~,~,hessian] = fminunc(fun, x_star, params.optimoptions);

% store the covariance matrix of x_(k-M+1)
obj.PX_prior= inv( hessian(end-2*params.m+1:end-params.m, end-2*params.m+1:end-params.m) );

% from a vector to cells 
from_vector_to_estimator(obj, x_star, params)


end



