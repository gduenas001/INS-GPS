
clear; clc; close all;

fileLIDAR= strcat('../../data/vehicle/20190110/LIDAR/');

firstEpoch= 0;
lastEpoch= 3077;

% Initial rotation to get: x=foward & z=down
R_init= [ 0, -1;
         -1, 0];

for epoch= firstEpoch:lastEpoch
    fprintf('Epoch: %d\n', epoch)
    
    fileName= strcat(fileLIDAR, 'textFiles/Epoch', num2str(epoch), '.txt');
    z= importdata(fileName);
    
    if ~isempty(z)
        z= ( R_init * z' )';
    end
    save(strcat(fileLIDAR,'matFiles/Epoch',num2str(epoch),'.mat'),'z');
end

     


























