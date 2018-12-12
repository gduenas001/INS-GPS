
clear; close all; clc;



% load DATA
load('DATA_M1_range20.mat');
P_HMI_M1= DATA.P_HMI_save;

load('DATA_M2_range20.mat');
P_HMI_M2= DATA.P_HMI_save;

load('DATA_M3_range20.mat');
P_HMI_M3= DATA.P_HMI_save;


load('DATA_M1_range25.mat');
P_HMI_M1_25= DATA.P_HMI_save;

load('DATA_M2_range25.mat');
P_HMI_M2_25= DATA.P_HMI_save;



% paramters
num_epochs= length(P_HMI_M1);
start_epoch= 1;
end_epoch= num_epochs-5;

figure; hold on; grid on;
plot(start_epoch:end_epoch, P_HMI_M1(start_epoch:end_epoch), 'linewidth', 2)
plot(start_epoch:end_epoch, P_HMI_M1_25(start_epoch:end_epoch), 'linewidth', 2)
plot(start_epoch:end_epoch, P_HMI_M2(start_epoch:end_epoch), 'linewidth', 2)
plot(start_epoch:end_epoch, P_HMI_M2_25(start_epoch:end_epoch), 'linewidth', 2)
plot(start_epoch:end_epoch, P_HMI_M3(start_epoch:end_epoch), 'linewidth', 2)
set(gca,'Yscale','log');
legend('M = 1 (20)', 'M = 1 (25)', 'M = 2 (20)', 'M = 2 (25)', 'M = 3 (20)')

