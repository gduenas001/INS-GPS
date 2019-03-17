function calibration(obj, imu_msmt, params)
% create a fake msmt and do a KF update to calibrate biases.
% Also increase D_bar so that biases keep changing (not too
% small variance)

% create a fake msmt and make a KF update
z= [zeros(6,1); obj.initial_attitude];

% Calibration msmt update
L= obj.PX(1:15,1:15) * params.H_cal' /...
    (params.H_cal*obj.PX(1:15,1:15)*params.H_cal' + params.R_cal);
z_hat= params.H_cal * obj.XX(1:15);
innov= z - z_hat;
innov(end)= pi_to_pi(innov(end));
obj.XX(1:15)= obj.XX(1:15) + L*innov;
obj.PX(1:15,1:15)= obj.PX(1:15,1:15) - L * params.H_cal * obj.PX(1:15,1:15);

% linearize and discretize after every non-IMU update
obj.linearize_discretize( imu_msmt, params.dt_imu, params);

% If GPS is calibrating initial biases, increse bias variance
obj.D_bar(10:12,10:12)= obj.D_bar(10:12,10:12) +...
    diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
obj.D_bar(13:15,13:15)= obj.D_bar(13:15,13:15) +...
    diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
end