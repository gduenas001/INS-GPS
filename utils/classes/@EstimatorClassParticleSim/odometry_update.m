function odometry_update(obj, params)

% velocity & steering angle
vel= params.velocity_sim;
phi= obj.steering_angle;
% phi= params.steering_angle_sim;

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
             obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
             pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

% Add noise to the controls
vel= vel + normrnd(0, params.sig_velocity_sim);
phi= phi + normrnd(0, params.sig_steering_angle_sim);

for i=1:size(obj.XX_particles,1)
    obj.XX_particles(i,:)= obj.return_odometry_update(obj.XX_particles(i,:), [vel; phi], params);
end

obj.XX =mean(obj.XX_particles)';
obj.PX =cov(obj.XX_particles);

% save the velocity and steering angle 
obj.odometry_k= [vel; phi];

end
