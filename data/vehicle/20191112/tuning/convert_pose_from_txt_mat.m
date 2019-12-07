clear
close all
clc

% open and read the text file
data= table2array(readtable('pose.txt'));
save('pose.mat','data');
