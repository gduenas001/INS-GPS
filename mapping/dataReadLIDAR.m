
function z= dataReadLIDAR(epoch, params)
% load the mat file with the extrated features at the lidar epoch specified

fileName= strcat(params.file_name_lidar_path,'matFiles/Epoch',num2str(epoch),'.mat');

% loads the z variable with range and bearing
load(fileName);

if params.SWITCH_REMOVE_FAR_FEATURES, z= removeFarFeatures(z, params.lidarRange); end

% Add height
height= 1.5;
z= [z, ones(size(z,1),1)*height];

end


function [z]= removeFarFeatures(z,lidarRange)
% removes extracted features farther than "lidarRange"

d= z(:,1).^2 + z(:,2).^2;

z( d > lidarRange^2, :)= [];  
    
end










