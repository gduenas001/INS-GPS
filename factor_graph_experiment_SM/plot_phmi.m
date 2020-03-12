
clear; close all; clc;

I_REQ= 1e-7;

% load data
load('fg_M30_nogps.mat');
figure; hold on; grid on;

% plot P(HMI)
plot(data_obj.im.time, ones(length(data_obj.im.time), 1) * I_REQ, ':', 'linewidth', 2)
plot(data_obj.im.time, data_obj.im.p_hmi, 'linewidth', 2)
xlim([0,230])
ylabel('P(HMI)','interpreter', 'latex','fontsize', 10)
set(gca, 'YScale', 'log')
ylim([1e-9,1]);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% --------------------------------------------------

availability= sum(data_obj.im.p_hmi < I_REQ) / length(data_obj.im.p_hmi);
fprintf('Availability: %d %%\n', round(availability*100))

% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.5 4];
% file_name= strcat('eps_vs_q');
% print(file_name,'-dpdf','-r0')