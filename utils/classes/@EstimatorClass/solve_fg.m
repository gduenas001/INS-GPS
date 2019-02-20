
function solve_fg(obj, params)

% total number of states to estimate
obj.m_M= (params.M + 1) * params.m;

% total number of measurements
obj.n_total= length(obj.z_fg);

% solve the problem
% fminunc();


end



