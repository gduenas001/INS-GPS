
function z= dataReadLIDAR(fileLIDAR, lidarRange, epoch, SWITCH_REMOVE_FAR_FEATURES)
% load the mat file with the extrated features at the lidar epoch specified

fileName= strcat(fileLIDAR,'matFiles/Epoch',num2str(epoch),'.mat');

% loads the z variable with range and bearing
load(fileName);

if SWITCH_REMOVE_FAR_FEATURES, z= removeFarFeatures(z, lidarRange); end

% Add height
height= 1.5;
z= [z, ones(size(z,1),1)*height];

end


function [z]= removeFarFeatures(z,lidarRange)
% removes extracted features farther than "lidarRange"

d= z(:,1).^2 + z(:,2).^2;

z( d > lidarRange^2, :)= [];  
    
end










