
function plot_map_localization_sim(obj, estimator, num_readings, params)

% Plot GPS+IMU estimated path
figure; hold on; grid on;

% for factor graphs --> plot true x instead of estimated
if params.SWITCH_OFFLINE
    plot(obj.update.x_true(1,:), obj.update.x_true(2,:), 'k.','markersize', 4);
else 
    plot(obj.pred.XX(1,:), obj.pred.XX(2,:), 'b-');
    plot(obj.update.XX(1,:), obj.update.XX(2,:), 'g.','markersize', 7);
end

if ~isempty(obj.gps_msmts)
    plot(obj.gps_msmts(:,1), obj.gps_msmts(:,2), 'r*', 'markersize', 4);
end

% create a map of landmarks
lm_map= [estimator.landmark_map(:,1),...
    estimator.landmark_map(:,2),...
    zeros(estimator.num_landmarks,1)];
plot(lm_map(:,1), lm_map(:,2), 'b+', 'markersize',2.5);

if ~isempty(obj.msmts)
    plot(obj.msmts(:,1), obj.msmts(:,2), 'k.');
end

plot(obj.update.x_true(1,1), obj.update.x_true(2,1), 'x', 'Color',[0.9290 0.6940 0.1250], 'linewidth', 2);
plot(obj.update.x_true(1,1), obj.update.x_true(2,1), '.', 'Color',[0.9290 0.6940 0.1250], 'linewidth', 3);

% plot attitude every 50 IMU readings
for i= 1:num_readings
    if rem(i,50) == 0
        R_NB= R_NB_rot( 0, 0, obj.pred.XX(3,i));
        xyz_N= R_NB*params.xyz_B + obj.pred.XX(:,i);
        plot(xyz_N(1,:), xyz_N(2,:), 'g-', 'linewidth', 2);
    end
end
set(gca,'FontSize',10)
xlabel('X [m]','FontSize',10); ylabel('Y [m]','FontSize',10); zlabel('Z [m]','FontSize',10);
%axis equal

end