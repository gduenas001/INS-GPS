
function generate_gyro_msmt_sim(obj, theta_prev, theta_next, params)

% true gyro msmt
obj.z_gyro= ( theta_next - theta_prev ) / params.dt_sim;

% add noise
obj.z_gyro= obj.z_gyro + normrnd(0, params.sig_gyro_z);

end






