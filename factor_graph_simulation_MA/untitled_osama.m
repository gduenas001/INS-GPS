a=-100:1:100;
b=zeros(size(a));
for i = 1:length(a)
    b(i)= obj.optimization_fn(a(i), fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M );
end

plot(a,b)