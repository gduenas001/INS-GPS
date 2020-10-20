function [c,ceq]= constriant_fn(obj, f_hat_sig_hat_f_h, params)
    %Cov_prior_hat = [f_hat_sig_hat_f_h(params.m+1), f_hat_sig_hat_f_h(params.m+2), f_hat_sig_hat_f_h(params.m+3);
    %                 f_hat_sig_hat_f_h(params.m+2), f_hat_sig_hat_f_h(params.m+4), f_hat_sig_hat_f_h(params.m+5);
    %                 f_hat_sig_hat_f_h(params.m+3), f_hat_sig_hat_f_h(params.m+5), f_hat_sig_hat_f_h(params.m+6)];
    %c = -eig(Cov_prior_hat)'*100000000;
    %b=zeros(params.m,1);
    %A=zeros(params.m,length(f_hat_sig_hat_f_h_init));
    %A(:,params.m+1:2*params.m) = eye(params.m)*-1;
    c=[-f_hat_sig_hat_f_h(params.m+1:2*params.m)'];% %;-(f_hat_sig_hat_f_h(end)-obj.eta_k_min);(f_hat_sig_hat_f_h(end)-obj.eta_k_max)];
    ceq=[];
end