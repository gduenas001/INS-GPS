
clear all; close all; clc;

% load data
load('../data/vehicle/20190110/FG.mat')
data= FG; clear FG;
pose= data.pose;
gps_R= data.gps_R;
gps_msmt= data.gps_msmt;
associations= data.associations;

% number of epochs
num_epochs= length(pose);

pose_gps= zeros(2,num_epochs);
sig_gps= zeros(2,num_epochs);
n_l= zeros(num_epochs, 1);
for i= 1:num_epochs
    
    if ~isempty(associations{i})
        n_l(i)= length(associations{i});
    end
    
    if ~isempty(gps_R{i})
        sig_gps(:,i)= sqrt( gps_R{i}(1:2) );
    end
    
    if ~isempty(gps_msmt{i})
        pose_gps(:,i)= gps_msmt{i}(1:2);
    end
    
end

figure; hold on; grid on;
plot(1:num_epochs, n_l, '-')

figure; hold on; grid on;
plot(1:num_epochs, sig_gps(1,:), '-')
plot(1:num_epochs, sig_gps(2,:), '-')
% plot(1:num_epochs, pose_gps(1,:), '-')