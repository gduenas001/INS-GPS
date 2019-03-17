function yaw_update(obj, w, params)


n_L= (length(obj.XX) - 15) / 2;
H= zeros(1, 15 + 2*n_L);
H(9)= 1;

z= obj.yawMeasurement(w, params);

R= params.R_yaw_fn( norm(obj.XX(4:6)));
L= obj.PX*H' / (H*obj.PX*H' + R);
innov= z - H*obj.XX;
innov= pi_to_pi(innov);
obj.XX(9)= pi_to_pi(obj.XX(9));
obj.XX= obj.XX + L*innov;
obj.PX= obj.PX - L*H*obj.PX;
end