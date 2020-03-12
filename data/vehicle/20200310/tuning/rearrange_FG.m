clear
close all
clc

load('SM.mat')

pose=data;

%load('landmark_map_old.mat')

load('LIDAR/T_LIDAR.mat')

load('IMU/IMU.mat')
IMU=data;
clear data
%data.imu=cell(size(T_LIDAR,1),1);
data.lidar=cell(size(T_LIDAR,1),1);
data.pose=cell(size(T_LIDAR,1),1);
%psi_offset=-90*pi/(180);
%R_LM_map_2D= [cos(pi),-sin(pi);sin(pi),cos(pi)];
%R_LM_map_3D= [cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%landmark_map= (R_LM_map_2D*landmark_map')';
%ang_deg= -1.6;
%rot_offset= [cos(ang_deg*pi/180),sin(ang_deg*pi/180);-sin(ang_deg*pi/180),cos(ang_deg*pi/180)];
%landmark_map= (rot_offset*landmark_map')';
%landmark_map(:,1)= landmark_map(:,1);%+6;
%landmark_map(:,2)= landmark_map(:,2);%-73;
%landmark_map(:,2)=-landmark_map(:,2);
%save('landmark_map.mat','landmark_map')
%R_psi_2D= [cos(psi_offset),-sin(psi_offset);sin(psi_offset),cos(psi_offset)];
%R_psi_3D= [cos(psi_offset),-sin(psi_offset),0;sin(psi_offset),cos(psi_offset),0;0,0,1];
for i=0:(size(T_LIDAR,1)-1)
    load(['LIDAR/matFiles/Epoch',num2str(i),'.mat']);
    if ~isempty(z)
        %z_new=(R_LM_map_2D*R_psi_2D*([z(:,1),z(:,2)]'))';
        z_new=[z(:,1),z(:,2)];
    else
        z_new=[];
    end
    data.lidar{i+1}=z_new;
    [~,j]=min(abs(pose(:,1)-T_LIDAR(i+1,2)));
    data.pose{i+1}=pose(j,2:10)';%,zeros(1,6)]';
    [~,j]=min(abs(IMU(:,4)-T_LIDAR(i+1,2)));
    %data.imu{i}=[R_LM_map_3D*R_psi_3D,zeros(3);zeros(3),R_LM_map_3D*R_psi_3D]*(IMU(j,5:10)');
end
save('FG.mat','data')