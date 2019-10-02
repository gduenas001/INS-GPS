clear
close all
clc

load('pose.mat')

pose=data;

load('landmark_map.mat')

load('LIDAR/T_LIDAR.mat')

load('IMU/IMU.mat')
IMU=data;
clear data
data.imu=cell(size(T_LIDAR,1),1);
data.lidar=cell(size(T_LIDAR,1),1);
data.pose=cell(size(T_LIDAR,1),1);

for i=1:size(T_LIDAR,1)
    load(['LIDAR/matFiles/Epoch',num2str(i-1),'.mat']);
    data.lidar{i}=z;
    [~,j]=min(abs(pose(:,1)-T_LIDAR(i,2)));
    data.pose{i}=[pose(j,2:10),zeros(1,6)]';
    [~,j]=min(abs(IMU(:,4)-T_LIDAR(i,2)));
    data.imu{i}=IMU(j,5:10);
end
save('FG.mat','data')