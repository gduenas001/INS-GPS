clear all; close all; clc;

% select map ID
density= 1;
map_i= 2;

% load data
file_name_map= 'offline_exp.mat';
data= load(file_name_map);
params= data.params;
data_obj= data.data_obj;
estimator= data.estimator;

% figure
figure; hold on;
subplot(2,1,1); hold on; box on;

% plot map
lm_map= [estimator.landmark_map(:,1), estimator.landmark_map(:,2)];
plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize', 12);

% plot path
plot(data_obj.update.x_true(1,1:end-1), data_obj.update.x_true(2,1:end-1), 'b-', 'linewidth', 2);

% axis limits
axis equal
x_limits= xlim;
y_limits= ylim;
% ylim(y_limits+70)


% axis labels
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)


% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 5];
% file_name= strcat('path_', num2str(density), '_', num2str(map_i));
file_name= 'path_exp';
print(file_name,'-dpdf','-r0')

