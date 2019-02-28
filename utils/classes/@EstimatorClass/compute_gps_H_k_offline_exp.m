function compute_gps_H_k_offline_exp(obj, params, FG, epoch)

if isempty(FG.gps_R{epoch}) || (~params.SWITCH_GPS_FG)
    
    obj.n_gps_k= 0;
    obj.H_k_gps= [];
    
else
    
    R= [params.mult_factor_pose_gps*eye(3),zeros(3);zeros(3),params.mult_factor_vel_gps*eye(3)] * diag( FG.gps_R{epoch} );
    obj.H_k_gps= sqrtm( inv(R) ) * [eye(6), zeros(6,9)];
    obj.n_gps_k= 6;
    
end
end