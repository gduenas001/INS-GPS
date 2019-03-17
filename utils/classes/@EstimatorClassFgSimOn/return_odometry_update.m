
function x= return_odometry_update(obj, x, u, params)
% u: [velocity; steering angle]

vel= u(1);
phi= u(2);

% True State
x= [x(1) + vel * params.dt_sim * cos(phi + x(3));
    x(2) + vel * params.dt_sim * sin(phi + x(3));
    pi_to_pi( x(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

end



