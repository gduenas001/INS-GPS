close all;


x_vals= 0:0.01:10;
num_x= length(x_vals);

out= [];
for i= 1:num_x
    out(i)= -obj.optimization_fn(...
        x_vals(i), fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps );
end


figure; hold on; grid on;
plot(x_vals, out, '-')

x_vals= 0:0.05:200;
num_x= length(x_vals);

out= [];
for i= 1:num_x
    out(i)= -obj.optimization_fn(...
        x_vals(i), fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps );
end


figure; hold on; grid on;
plot(x_vals, out, '-')


%%

x_vals= 1:1:300;
num_x= length(x_vals);

error_prob= [];
for i= 1:num_x
    error_prob(i)= (1 - normcdf(0.5 , x_vals(i) * fx_hat_dir, obj.sigma_hat) +...
                normcdf(-0.5 , x_vals(i) * fx_hat_dir, obj.sigma_hat));
end

figure; hold on; grid on;
plot(x_vals, error_prob, '-')

non_detect_prob= [];
for i= 1:num_x
    non_detect_prob(i)= ncx2cdf(obj.T_d,  obj.n_M + obj.n_M_gps , x_vals(i).^2 * M_dir );
end

figure; hold on; grid on;
plot(x_vals, non_detect_prob, '-')

