
function z= dataReadLIDAR(fileLIDAR, lidarRange, epoch, SWITCH_REMOVE_FAR_FEATURES)

fileName= strcat(fileLIDAR,'matFiles/Epoch',num2str(epoch),'.mat');

load(fileName);

if SWITCH_REMOVE_FAR_FEATURES, z= removeFarFeatures(z, lidarRange); end

% Add height
height= 1.5;
z= [z, ones(size(z,1),1)*height];


end


function [z]= removeFarFeatures(z,lidarRange)
    d= z(:,1).^2 + z(:,2).^2;
    
    z( d > lidarRange^2, :)= [];
    
    
end










