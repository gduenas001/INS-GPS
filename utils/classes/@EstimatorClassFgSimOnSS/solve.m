
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

%initializaing SS RAIM parameters
obj.solns= inf*ones(obj.n_L_M); %state of interest estimate for every hypothesis (excluding the faulted msmts)
obj.test_statistics= inf*ones(obj.n_L_M); %SS RAIM test statistics
obj.sigma_hat_j= inf*ones(obj.n_L_M); %std dev of every solution (excluding the faulted msmts)
obj.T_delta_j= inf*ones(obj.n_L_M);  %Thresholds for every test statistic
obj.availability=1;
% perform the optimization for every hypothesis including the null (full solution)
% Assuming that only one LM is faulted
for i= 0:obj.n_L_M
    
    if i==1
        tic
    end

    % create optimization function for hypothesis i
    fun= @(x) obj.optimization_fn(x, params, i);

    % from cells to vectors
    x_star= obj.from_estimator_to_vector(params);

    % saves the prior separately
    obj.x_prior= obj.x_ph{obj.M};

    % solve the problem
    % [residual, grad, A, b]= obj.optimization_fn(x_star, params, i);

    % solve the problem
    [x_star, ~,~,~,~,hessian] = fminunc(fun, x_star, params.optimoptions);
    
    if i ~= 0 % evaluating the test statistic and threshold for the ith hypothesis (faulted hypothesis)
        
        alpha= build_state_of_interest_extraction_matrix(obj, params, x_star);
        
        obj.solns(i)=alpha'*x_star;
        
        obj.sigma_hat_j(i) = sqrt( alpha' / hessian * alpha );
        
        sigma_hat_delta_j = sqrt( obj.sigma_hat_j(i)^2 - obj.sigma_hat_full^2 );
        
        obj.test_statistics(i)= obj.x_hat-obj.solns(i);
        
        obj.T_delta_j(i)= norminv( 1 - params.continuity_requirement/(2*obj.n_L_M) ) * sigma_hat_delta_j;
        
        if obj.test_statistics(i) > obj.T_delta_j(i)
            obj.availability=0;
        end
        
    else % evaluating the full solution for the null hypothsis (no faults)
        
        hessian_full= hessian;
        
        x_full= x_star;
        
        alpha= build_state_of_interest_extraction_matrix(obj, params, x_full);
        
        obj.x_hat= alpha'*x_full;
        
        obj.sigma_hat_full= sqrt( alpha' / hessian_full * alpha );
        
    end
    
    % multiply by two so that it fits the non-central chi-squared
    % obj.q_d= obj.q_d * 2;

    % compute detector threshold
    % obj.T_d= chi2inv( 1- params.continuity_requirement, obj.n_total - obj.m_M );

    % debuggin points to check jacobian
    % [residual, grad, A, b]= obj.optimization_fn_fg(x_star, params);
    % dif= inv(A'*A) - inv(hessian);
    % dif( abs(dif) < 5 )= 0;
    % diag(dif)

end

obj.detector_elapsed_time =toc;
% from a vector to cells 
from_vector_to_estimator(obj, x_full, params);

% store the prior, x_(k-M+1), as a future msmt
% hessian= A' * A;

obj.PX= inv( hessian_full );
obj.PX_prior= obj.PX( params.m + 1 : 2 * params.m , params.m + 1 : 2 * params.m );
obj.PX= obj.PX( end - params.m + 1 : end , end - params.m + 1 : end  );
% obj.Gamma_prior= inv( PX(params.m+1:2*params.m, params.m+1:2*params.m) );

end



