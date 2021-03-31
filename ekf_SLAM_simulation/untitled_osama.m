a=-10:0.1:10;
b=zeros(size(a));
for i = 1:length(a)
    b(i)= obj.optimization_fn(a(i), fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, params.m_F * obj.n_L_k );
end
plot(a,b)
set(gca, 'YScale', 'log')