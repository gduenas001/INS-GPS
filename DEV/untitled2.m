clear
% close all
clc
fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180725/LIDAR/');
% fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/LIDAR/');
figure()
hold on
r=[];
for i = 1:1600
    z= dataReadLIDAR(fileLIDAR, 25, i, 0);
    r=[r;z];
    zbody= removeFeatureInArea([zeros(8,1);deg2rad(-90)], z, -35, -5, 15, 30);
    zbody= removeFeatureInArea([zeros(8,1);deg2rad(-90)], zbody, 2, 9, -45, 15);
    zbody= removeFeatureInArea([zeros(8,1);deg2rad(-90)], zbody, 7, 18, 10, 20);
    % zbody= removeFeatureInArea(zeros(9,1), z, 0,8,0,15); % [zeros(8,1);deg2rad(180)]
    % zbody= removeFeatureInArea(zeros(9,1), zbody, -28,15,-24,-18);
    % zbody= removeFeatureInArea(zeros(9,1), zbody, -35,-27,27,30);
    plot(z(:,1),z(:,2),'g+', 'markersize',2);
    plot(zbody(:,1),zbody(:,2),'r+', 'markersize',2);
end
% plot(r(idx==20,1),r(idx==20,2),'r+', 'markersize',2);
% z= dataReadLIDAR(fileLIDAR, 50, 1770, 0);
% plot(z(:,1),z(:,2),'g+', 'markersize',2);