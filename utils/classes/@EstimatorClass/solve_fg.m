
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
x_star= [];
for i= params.M:-1:1
    x_star= [x_star; obj.x_ph{i}];
end
x_star= [ x_star; obj.XX ];

% solve the problem
[x_star,~,~,~,~,hessian] = fminunc(fun, x_star, params.optimoptions);

% store the covariance matrix of x_(k-M+1)
obj.PX_prior= inv( hessian(end-2*params.m+1:end-params.m, end-2*params.m+1:end-params.m) );

% from a vector to cells 
inds= 1:params.m;
for i= params.M:-1:1
    % update the cell
    obj.x_ph{i}= x_star(inds);
    
    % update index
    inds= inds + params.m;
end
% current pose
obj.XX= x_star(end - params.m + 1:end);


end



