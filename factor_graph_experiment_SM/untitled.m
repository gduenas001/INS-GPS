x= -1000:1000;
y=inf*ones(length(x));
for i= 1:length(x)
    y(i)= obj.optimization_fn(x(i), fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps );
end
plot(x,y)


[f_M_mag_out_5, p_hmi_H_5]= fminbnd( @(f_M_mag) obj.optimization_fn(...
            f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps ),...
            -10, 10);
        
obj.optimization_fn(0, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps )