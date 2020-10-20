function odometry_update(obj, params)

% % velocity & steering angle
% vel= params.velocity_sim;
% phi= obj.steering_angle;

% phi= params.steering_angle_sim;

%[obj.Phi_k, obj.D_bar]= obj.return_Phi_and_D_bar(obj.XX_predict, vel, phi, params);

%obj.g_k_bar = obj.return_odometry_update(obj.XX_predict, [vel; phi], params);

obj.G_k_particles = inf*ones(params.m,params.m*size(obj.XX_particles_predict,1));
obj.F_k_particles = inf*ones(params.m*size(obj.XX_particles_predict,1),params.size_of_control_vector);

obj.Control_for_each_particle=inf*ones(2,size(obj.XX_particles_predict,1));

for i=1:size(obj.XX_particles_predict,1)
    % velocity & steering angle
    vel_i= params.velocity_sim + normrnd(0, params.sig_velocity_sim);
    phi_i= obj.steering_angle + normrnd(0, params.sig_steering_angle_sim);
    obj.Control_for_each_particle(:,i) = [vel_i; phi_i];
    [obj.G_k_particles(:,params.m*(i-1)+1:params.m*i), obj.F_k_particles(params.m*(i-1)+1:params.m*i,:)]= obj.return_G_and_F(obj.XX_particles_predict(i,:), vel_i, phi_i, params);
    obj.XX_particles_predict(i,:)= obj.return_odometry_update(obj.XX_particles_predict(i,:), [vel_i; phi_i], params);
end

obj.XX_predict =mean(obj.XX_particles_predict)';
%[~, indices_of_unique_particles, ~] = unique(obj.particles_indices_predict);
obj.particles_indices_predict = transpose( 1:size(obj.XX_particles_predict,1) );
%obj.SX_predict = cov(obj.XX_particles_predict(indices_of_unique_particles,:));
obj.SX_predict = cov(obj.XX_particles_predict);

% Add noise to the controls
vel= params.velocity_sim + normrnd(0, params.sig_velocity_sim);
phi= obj.steering_angle + normrnd(0, params.sig_steering_angle_sim);

% True State
obj.x_true= [obj.x_true(1) + vel * params.dt_sim * cos(phi + obj.x_true(3));
             obj.x_true(2) + vel * params.dt_sim * sin(phi + obj.x_true(3));
             pi_to_pi( obj.x_true(3) + vel * params.dt_sim * sin(phi) / params.wheelbase_sim )];

% save the velocity and steering angle 
obj.odometry_k= [vel; phi];

end
