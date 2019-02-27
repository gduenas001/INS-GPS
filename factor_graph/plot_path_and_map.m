clear all; close all; clc;

% select map ID
density= 1;
map_i= 2;

% load data
path= strcat('../data/simulation/factor_graph/results/density_00', num2str(density), '_fix/map_');
file_name_map= strcat( path, num2str(map_i), '/offline.mat' );
data= load(file_name_map);
params= data.params;
data_obj= data.data_obj;
estimator= data.estimator;

% figure
figure; hold on; grid on;

% create a map of landmarks
lm_map= [estimator.landmark_map(:,1),...
        estimator.landmark_map(:,2),...
        zeros(estimator.num_landmarks,1)];
plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize', 15);

% plot true path
plot(data_obj.update.x_true(1,:), data_obj.update.x_true(2,:), 'b-','markersize', 7, 'linewidth', 2);

% plot waypoints
plot(params.way_points(1,:), params.way_points(2,:), 'gp', 'markersize', 10, 'MarkerFaceColor', 'g');

% axis limits
axis equal
xlim([-25,220])
% ylim([-45,45])


% axis labels
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)


fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 3];
file_name= strcat('path_00', num2str(density), '_', num2str(map_i));
print(file_name,'-dpdf','-r0')

% close all;