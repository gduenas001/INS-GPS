
function gps_update_sim(obj, params)

% simulate measurement
z= obj.x_true + mvnrnd(zeros(1,3), params.R_gps_sim);
H= eye(3);

L= obj.PX*H' / (H*obj.PX*H' + params.R_gps_sim);
innov= z - H*obj.XX;

obj.XX(3)= pi_to_pi( obj.XX(3) );
obj.XX= obj.XX + L*innov;
obj.PX= obj.PX - L*H*obj.PX;



end