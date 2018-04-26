
function z= dataReadLIDAR(fileLIDAR, epoch)

fileName= strcat(fileLIDAR,'matFiles/Epoch',num2str(epoch),'.mat');

load(fileName);

% Add the z component
height= 0;
z= [z, -height*ones(size(z,1),1)];













