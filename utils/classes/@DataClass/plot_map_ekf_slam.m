function lm_map= plot_map_ekf_slam(obj, estimator, params)

% Plot GPS+IMU estimated path

figure; hold on; grid on;
plot(obj.update.XX(1,:), obj.update.XX(2,:),'b.');
plot(obj.update.x_true(1,:), obj.update.x_true(2,:),'r.');
if estimator.num_of_estimated_landmarks > 0 % Plot landmarks
    % create a map of landmarks
    lm_map= [estimator.XX(params.m+1:2:end), estimator.XX(params.m+2:2:end)];
    lm_to_eliminate= [];
    for i= 1:estimator.num_of_estimated_landmarks
        if estimator.appearances(i) < params.min_appearances
            lm_to_eliminate= [lm_to_eliminate; i];
        end
    end
    lm_map(lm_to_eliminate,:)= [];
    
    % plot
    plot(lm_map(:,1), lm_map(:,2), 'g+', 'markersize',20);
    plot(obj.msmts(:,1), obj.msmts(:,2), 'k.');
end

% create a map of landmarks
lm_map_GT= [estimator.landmark_map(:,1),...
    estimator.landmark_map(:,2)];
plot(lm_map_GT(:,1), lm_map_GT(:,2), 'c+', 'markersize',2.5);

xlabel('x [m]'); ylabel('y [m]');
axis equal

end