function yaw= yawMeasurement(obj, w, params)

r= [-params.r_IMU2rearAxis; 0; 0];
v_o= obj.XX(4:6);
R_NB= R_NB_rot( obj.XX(7), obj.XX(8), obj.XX(9));

v_a= v_o + R_NB * cross(w,r);
v_a= v_a / norm(v_a);

yaw= atan2(v_a(2),v_a(1));
end