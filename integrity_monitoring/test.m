
n= 50;

tic
for count= 1:n 
    [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag)...
        -((1-   normcdf(params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat)   +...
        normcdf(-params.alert_limit, f_M_mag * fx_hat_dir, sigma_hat))...
        * ncx2cdf(obj.detector_threshold, params.m_F * obj.n_L_M, f_M_mag^2 * M_dir )),...
        -5, 5);
end
toc

%%
tic
for count= 1:n
    [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
        f_M_mag, fx_hat_dir, M_dir, sigma_hat, params.alert_limit, params.m_F * obj.n_L_M),...
        -5, 5);
end
toc