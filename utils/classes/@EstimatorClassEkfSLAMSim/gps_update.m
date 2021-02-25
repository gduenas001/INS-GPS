
function gps_update(obj, z, params)

% build matrices & vectors
H= [eye(2),zeros(2,1)];
L= obj.PX*H' / (H*obj.PX*H' + params.R_gps_sim);
innov= z - H*obj.XX;

% udpate
obj.XX(3)= pi_to_pi( obj.XX(3) );
obj.XX= obj.XX + L*innov;
obj.PX= obj.PX - L*H*obj.PX;

end