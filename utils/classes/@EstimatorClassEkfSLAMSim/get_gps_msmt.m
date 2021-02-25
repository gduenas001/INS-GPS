function z= get_gps_msmt(obj, params)
% simulate measurement
z= obj.x_true(1:2) + mvnrnd(zeros(2,1), params.R_gps_sim)';
end