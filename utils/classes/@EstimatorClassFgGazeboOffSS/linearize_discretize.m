function linearize_discretize(obj, u, dT, params)
% updates Phi_k & D_bar

if params.SWITCH_CALIBRATION
    taua= params.taua_calibration;
    tauw= params.tauw_calibration;
    S= params.S_cal;
else
    taua= params.taua_normal_operation;
    tauw= params.tauw_normal_operation;
    S= params.S;
end

% Compute the F and G matrices (linear continuous time)
[F,G]= FG_fn(u(1),u(2),u(3),u(5),u(6),...
    obj.XX(7),obj.XX(8),obj.XX(9),0,0,0,0,0,... %,obj.XX(10),obj.XX(11),obj.XX(12),obj.XX(14),obj.XX(15),...
    taua,tauw);

% Discretize system for IMU time (only for variance calculations)
obj.discretize(F, G, S, dT);
end