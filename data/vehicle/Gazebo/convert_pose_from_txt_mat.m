clear
close all
clc

% open and read the text file
data= table2array(readtable('pose.txt'));
%data(:,2) = 0*ones(size(data(:,2)));
%data(:,10) = -1.57*ones(size(data(:,10)));
data(:,1) = data(:,1)-3.71e7; %3.72
save('pose.mat','data');
