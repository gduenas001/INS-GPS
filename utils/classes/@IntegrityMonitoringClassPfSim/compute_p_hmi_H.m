
function p_hmi_H= compute_p_hmi_H(obj, fault_ind, params, estimator)

tmp = estimator.SX_prior;%*1e1;%*1e2;%*1e3;%*1e4;
tmp1 = 0;%eps;%0.0000001;%1;

% build extraction matrix
if fault_ind == 0
    obj.compute_E_matrix( 0, params.m_F, estimator);
    %f_hat_sig_hat_f_h_init = [0,0,0,1,0,0,1,0,1];
    %f_hat_sig_hat_f_h_init = [0,0,0,estimator.prior_estimate_cov(1,1),estimator.prior_estimate_cov(1,2),estimator.prior_estimate_cov(1,3),estimator.prior_estimate_cov(2,2),estimator.prior_estimate_cov(2,3),estimator.prior_estimate_cov(3,3)];
    %f_hat_sig_hat_f_h_init = [eps,eps,eps,tmp(1,1),tmp(1,2),tmp(1,3),tmp(2,2),tmp(2,3),tmp(3,3)];
    f_hat_sig_hat_f_h_init = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3)];
    % % f_hat_sig_hat_f_h_init_eta_max = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3)]; % %,obj.eta_k_max];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
    % % f_hat_sig_hat_f_h_init_eta_min = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3)]; % %,obj.eta_k_min];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
    % % f_hat_sig_hat_f_h_init_eta_avg = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3)]; % %,(obj.eta_k_max+obj.eta_k_min)/2];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
else
    obj.compute_E_matrix( obj.inds_H{fault_ind}, params.m_F, estimator);
    %f_hat_sig_hat_f_h_init = [0,0,0,1,0,0,1,0,1,zeros(1,length(obj.inds_H{fault_ind})*params.m_F)];
    %f_hat_sig_hat_f_h_init = [0,0,0,estimator.prior_estimate_cov(1,1),estimator.prior_estimate_cov(1,2),estimator.prior_estimate_cov(1,3),estimator.prior_estimate_cov(2,2),estimator.prior_estimate_cov(2,3),estimator.prior_estimate_cov(3,3),zeros(1,length(obj.inds_H{fault_ind})*params.m_F)];
    %f_hat_sig_hat_f_h_init = [eps,eps,eps,tmp(1,1),tmp(1,2),tmp(1,3),tmp(2,2),tmp(2,3),tmp(3,3),eps*ones(1,length(obj.inds_H{fault_ind})*params.m_F)];
    f_hat_sig_hat_f_h_init = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3),tmp1*ones(1,length(obj.inds_H{fault_ind})*params.m_F)];
    % % f_hat_sig_hat_f_h_init_eta_max = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3),tmp1*ones(1,length(obj.inds_H{fault_ind})*params.m_F)]; % %,obj.eta_k_max];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
    % % f_hat_sig_hat_f_h_init_eta_min = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3),tmp1*ones(1,length(obj.inds_H{fault_ind})*params.m_F)]; % %,obj.eta_k_min];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
    % % f_hat_sig_hat_f_h_init_eta_avg = [tmp1,tmp1,tmp1,tmp(1,1),tmp(2,2),tmp(3,3),tmp1*ones(1,length(obj.inds_H{fault_ind})*params.m_F)]; % %,(obj.eta_k_max+obj.eta_k_min)/2];%,(obj.eta_k_min+obj.eta_k_max)/2,obj.eta_k_max];
end

%b=zeros(params.m,1);
%A=zeros(params.m,length(f_hat_sig_hat_f_h_init));
%A(:,params.m+1:2*params.m) = eye(params.m)*-1;

options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',inf,'ObjectiveLimit', -1.0000e+200,'MaxIterations',inf);

obj.optimization_scale = 1e0;

obj.optimization_scale = obj.optimization_fn(f_hat_sig_hat_f_h_init, params, estimator );
% % optimization_scale_eta_max = obj.optimization_fn(f_hat_sig_hat_f_h_init_eta_max, params, estimator );
% % optimization_scale_eta_min = obj.optimization_fn(f_hat_sig_hat_f_h_init_eta_min, params, estimator );
% % optimization_scale_eta_avg = obj.optimization_fn(f_hat_sig_hat_f_h_init_eta_avg, params, estimator );

% % if ( (optimization_scale_eta_max <= optimization_scale_eta_min) && (optimization_scale_eta_max <= optimization_scale_eta_avg) )
% %     f_hat_sig_hat_f_h_init = f_hat_sig_hat_f_h_init_eta_max;
% %     obj.optimization_scale = optimization_scale_eta_max;
% % elseif ( (optimization_scale_eta_min <= optimization_scale_eta_max) && (optimization_scale_eta_min <= optimization_scale_eta_avg) )
% %     f_hat_sig_hat_f_h_init = f_hat_sig_hat_f_h_init_eta_min;
% %     obj.optimization_scale = optimization_scale_eta_min;
% % else
% %     f_hat_sig_hat_f_h_init = f_hat_sig_hat_f_h_init_eta_avg;
% %     obj.optimization_scale = optimization_scale_eta_avg;
% % end

if (obj.optimization_scale==0)
    obj.optimization_scale = -1/(-0.1);
elseif (obj.optimization_scale>= -1e-250)
    obj.optimization_scale = -1/(-1e-250);
else
    obj.optimization_scale = -1/obj.optimization_scale;
end

[f_hat_sig_hat_f_h_optimal, p_hmi_H]= fmincon( @(f_hat_sig_hat_f_h) obj.optimization_fn(...
             f_hat_sig_hat_f_h, params, estimator ),...
             f_hat_sig_hat_f_h_init,[],[],[],[],[],[],@(f_hat_sig_hat_f_h) obj.constriant_fn(f_hat_sig_hat_f_h, params),options);%,[],[],[],[],[],[],@(f_hat_sig_hat_f_h) obj.constriant_fn(f_hat_sig_hat_f_h, params));
p_hmi_H = -p_hmi_H/obj.optimization_scale;

%opts = optimoptions(@fmincon,'Algorithm','sqp');

%problem = createOptimProblem('fmincon','objective',@(f_hat_sig_hat_f_h) obj.optimization_fn( f_hat_sig_hat_f_h, params, estimator ),'x0',f_hat_sig_hat_f_h_init,'nonlcon',@(f_hat_sig_hat_f_h) obj.constriant_fn(f_hat_sig_hat_f_h, params),'options',opts);
%problem = createOptimProblem('fmincon','objective',@(f_hat_sig_hat_f_h) obj.optimization_fn( f_hat_sig_hat_f_h, params, estimator ),'x0',f_hat_sig_hat_f_h_init,'Aineq',A,'bineq',b,'options',opts);

%gs = GlobalSearch;

%[f_hat_sig_hat_f_h_optimal, p_hmi_H] = run(gs,problem);

%p_hmi_H = -p_hmi_H/obj.optimization_scale;

f_prior_hat = f_hat_sig_hat_f_h_optimal(1:params.m)';
            
%Cov_prior_hat = [f_hat_sig_hat_f_h_optimal(params.m+1), f_hat_sig_hat_f_h_optimal(params.m+2), f_hat_sig_hat_f_h_optimal(params.m+3);
%                             f_hat_sig_hat_f_h_optimal(params.m+2), f_hat_sig_hat_f_h_optimal(params.m+4), f_hat_sig_hat_f_h_optimal(params.m+5);
%                             f_hat_sig_hat_f_h_optimal(params.m+3), f_hat_sig_hat_f_h_optimal(params.m+5), f_hat_sig_hat_f_h_optimal(params.m+6)];
Cov_prior_hat = diag(f_hat_sig_hat_f_h_optimal(params.m+1:2*params.m));

if ~isempty(obj.E)
    %E_f_k_h = f_hat_sig_hat_f_h_optimal(params.m*(params.m+3)/2+1:end)';
    E_f_k_h = f_hat_sig_hat_f_h_optimal(2*params.m+1:end)';
end

% % Worst-case fault direction
% f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A * obj.PX_M * alpha;
% f_M_dir= f_M_dir / norm(f_M_dir); % normalize

% worst-case fault magnitude
% fx_hat_dir= abs( (alpha' / obj.Gamma_fg) * obj.A' * f_M_dir );
% M_dir= abs( f_M_dir' * obj.M_M * f_M_dir );

% % save the interesting values for the optimization
% obj.counter_H= obj.counter_H + 1;
% obj.noncentral_dof{obj.counter_H}=  obj.n_M + obj.n_M_gps;
% obj.f_dir_sig2{obj.counter_H}= (fx_hat_dir / obj.sigma_hat)^2;
% obj.M_dir{obj.counter_H}= M_dir;

% p_hmi_H= 0;
% return




% % check if we should start evaluating f mag at zero
% if abs(obj.optimization_fn(...
%         0, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps )) > 1e-10
%     % worst-case fault magnitude
%     f_mag_min= 0;
%     f_mag_max= 5;
%     f_mag_inc= 5;
%     p_hmi_H_prev= -1;
%     for k= 1:10
%         [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%             f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps ),...
%             f_mag_min, f_mag_max);
%             
%         % make it a positive number
%         p_hmi_H= -p_hmi_H;
%         
%         % check if the new P(HMI|H) is smaller
%         if k == 1 || p_hmi_H_prev < p_hmi_H
%             p_hmi_H_prev= p_hmi_H;
%             f_mag_min= f_mag_min + f_mag_inc;
%             f_mag_max= f_mag_max + f_mag_inc;
%         else
%             p_hmi_H= p_hmi_H_prev;
%             break
%         end
%     end
%     % make a general optimization first
% else
%     p_hmi_H_1= 0;
%     p_hmi_H_2= 0;
%     p_hmi_H_3= 0;
%     p_hmi_H_4= 0;
%     f_M_mag_out_1= -10;
%     f_M_mag_out_2= -10;
%     f_M_mag_out_3= -10;
%     f_M_mag_out_4= -10;
%     
%     [f_M_mag_out_1, p_hmi_H_1]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%         f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
%         -10, 10);
%     p_hmi_H_1= -p_hmi_H_1;
%     if p_hmi_H_1 < 1e-10 || f_M_mag_out_1 > 8
%         [f_M_mag_out_2, p_hmi_H_2]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%             f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
%             -100, 100);
%         p_hmi_H_2= -p_hmi_H_2;
%         if p_hmi_H_2 < 1e-10 || f_M_mag_out_2 > 80
%             [f_M_mag_out_3, p_hmi_H_3]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                 f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
%                 -1000, 1000);
%             p_hmi_H_3= -p_hmi_H_3;
%             if p_hmi_H_2 < 1e-10 || f_M_mag_out_3 > 800
%                 [f_M_mag_out_4, p_hmi_H_4]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%                     f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
%                     -100000, 100000);
%                 p_hmi_H_4= -p_hmi_H_4;
%             end
%         end
%     end
%     [p_hmi_H,ind]= max( [p_hmi_H_1, p_hmi_H_2, p_hmi_H_3, p_hmi_H_4] );
% end
% 
% if ind==1
%     obj.f_M_mag_out= abs(f_M_mag_out_1);
% elseif ind==2
%     obj.f_M_mag_out= abs(f_M_mag_out_2);
% elseif ind==3
%     obj.f_M_mag_out= abs(f_M_mag_out_3);
% else
%     obj.f_M_mag_out= abs(f_M_mag_out_4);
% end
% 
% % worst-case fault magnitude
% %f_mag_min= 0;
% %f_mag_max= 5;
% %f_mag_inc= 5;
% %p_hmi_H_prev= -1;
% 
% %for k=1:10
% %    [obj.f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
% %        f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
% %        f_mag_min, f_mag_max);
% %
% %    % make it a positive number
% %    p_hmi_H= -p_hmi_H;
% 
%     % check if the new P(HMI|H) is smaller
% %    if k == 1 || p_hmi_H_prev < p_hmi_H
% %       p_hmi_H_prev= p_hmi_H;
% %       f_mag_min= f_mag_min + f_mag_inc;
% %       f_mag_max= f_mag_max + f_mag_inc;
% %    else
% %       p_hmi_H= p_hmi_H_prev;
% %       break
% %    end
% % end


end
