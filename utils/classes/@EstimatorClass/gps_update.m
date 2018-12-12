function gps_update(obj, z, R, params)


n_L= (length(obj.XX) - 15) / 2;

if norm(z(4:6)) > params.min_vel_gps && params.SWITCH_GPS_VEL_UPDATE % sense velocity
    R= diag( R );
    H= [eye(6), zeros(6,9), zeros(6,n_L*2)];
    disp('GPS velocity')
else
    z= z(1:3);
    R= diag( R(1:3) );
    H= [eye(3), zeros(3,12), zeros(3,n_L*2)];
    disp('-------- no GPS velocity ---------')
end

obj.XX(9)= pi_to_pi( obj.XX(9) );
L= obj.PX*H' / (H*obj.PX*H' + R);
innov= z - H*obj.XX;

obj.XX= obj.XX + L*innov;
obj.PX= obj.PX - L*H*obj.PX;
end