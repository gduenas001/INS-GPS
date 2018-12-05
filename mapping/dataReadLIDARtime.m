
function T_LIDAR= dataReadLIDARtime(fileName,timeInit)
% substracts the GPS start time which is used as reference

% loads the variable "T_LIDAR"
load(fileName); 

% Use the GPS first reading time as reference
T_LIDAR(:,2)= T_LIDAR(:,2) - timeInit;

% If some of the initial times are negative (prior to first GPS reading) --> eliminate them
T_LIDAR(T_LIDAR(:,2) < 0 , :)= [];






