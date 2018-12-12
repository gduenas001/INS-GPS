function imu_update( obj, imu_msmt, params )
% updates the state with the IMU reading, NO cov update

% Create variables (for clarity)
v= obj.XX(4:6);
phi= obj.XX(7); theta= obj.XX(8); psi= obj.XX(9);
b_f= obj.XX(10:12);
b_w= obj.XX(13:15);
f= imu_msmt(1:3);
w= imu_msmt(4:6);

if params.SWITCH_CALIBRATION
    taua= params.taua_calibration;
    tauw= params.tauw_calibration;
else
    taua= params.taua_normal_operation;
    tauw= params.tauw_normal_operation;
end

% Calculate parameters
R_NB= R_NB_rot (phi,theta,psi);
Q_BE= Q_BE_fn  (phi,theta);

r_dot= v;
v_dot= R_NB * ( f - b_f ) + params.g_N;
E_dot= Q_BE * ( w - b_w );
b_f_dot= -eye(3) / taua * b_f;
b_w_dot= -eye(3) / tauw * b_w;
x_dot= [r_dot; v_dot; E_dot; b_f_dot; b_w_dot];

% udpate estimate
obj.XX(1:15)= obj.XX(1:15) + params.dt_imu * x_dot;
obj.PX(1:15,1:15)= obj.Phi_k * obj.PX(1:15,1:15) * obj.Phi_k' + obj.D_bar;
end