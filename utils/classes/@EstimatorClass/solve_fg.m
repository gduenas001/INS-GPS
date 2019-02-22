
function solve_fg(obj, counters, params)


% total number of states to estimate
obj.m_M= (params.M + 1) * params.m;

% check that there are enough epochs
if counters.k_lidar <= params.M, return, end

% create optimization function 
fun= @(x) obj.optimization_fn_fg(x, params);

% from cells to vectors
x_star= obj.from_estimator_to_vector(params);

% saves the prior separately
obj.x_prior= obj.x_ph{params.M-1};

% solve the problem
[x_star,~,~,~,~,hessian] = fminunc(fun, x_star, params.optimoptions);

% debuggin points to check jacobian
% [~, A]= obj.optimization_fn_fg(x_star, params);
% dif= inv(A'*A) - inv(hessian);
% dif( abs(dif) < 5 )= 0;
% diag(dif)

% from a vector to cells 
from_vector_to_estimator(obj, x_star, params)

% store the prior, x_(k-M+1), as a future msmt
% hessian= A' * A;
obj.Gamma_prior= hessian(params.m+1:2*params.m, params.m+1:2*params.m);

end



