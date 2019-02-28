
clear; close all; clc;

% common paths for map and runs
path= strcat('../data/simulation/factor_graph/results/density_001_fix/map_');
figure; hold on; grid on;

% ------------------ offline data ------------------

% load data
map_i= 3;
file_name_map= strcat( path, num2str(map_i), '/offline.mat' );
load(file_name_map);

% save common x limits
x_limits= [data_obj.im.time(1), data_obj.im.time(end)];

% plot P(HMI)
subplot(3,1,1); hold on; 
plot(data_obj.im.time, ones(length(data_obj.im.time), 1) * 1e-5, ':', 'linewidth', 2)
plot(data_obj.im.time, data_obj.im.p_hmi, 'linewidth', 2)
grid on;
xlim(x_limits) % reset the x-axis (otherwise it moves)
set(gca,'XTickLabel',[])
ylabel('P(HMI)','interpreter', 'latex','fontsize', 10)
set(gca, 'YScale', 'log')
ylim([1e-12,1]);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% --------------------------------------------------


% ------------------ online data ------------------

% load data
run_i= 45;
file_name_run= strcat( path, num2str(map_i), '/online_', num2str(run_i), '.mat' );
load(file_name_run);

% plot error
subplot(3,1,2); hold on;
plot(data_obj.update.time, ones(length(data_obj.update.time), 1)*0.5,...
    ':', 'linewidth', 2, 'HandleVisibility','off')
plot(data_obj.update.time, abs( data_obj.update.error_state_interest(:) ),...
    '-', 'linewidth', 2)
grid on;
xlim(x_limits) % reset the x-axis (otherwise it moves)
set(gca,'XTickLabel',[])
ylabel('Error [m]','interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)

% plot detector
subplot(3,1,3); hold on;
plot(data_obj.update.time, data_obj.update.T_d, ':', 'linewidth', 2)
plot(data_obj.update.time, data_obj.update.q_d, '-', 'linewidth', 2)
grid on;
xlim(x_limits) % reset the x-axis (otherwise it moves)
ylim([0, 150])
xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
legend({'Detector, $q$', 'Threshold, $T$'}, 'interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% --------------------------------------------------




% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.5 4];
% file_name= strcat('eps_vs_q');
% print(file_name,'-dpdf','-r0')