
%==================================================================
filename= '224LM.txt';
file= csvimport(filename);
n_LM= cell2mat(file(2:end,:));
n_LM = [n_LM(:,1),n_LM(:,2)];


landmark_map2 = n_LM;
landmark_map2(:,1)= landmark_map2(:,1);%+6;%0
landmark_map2(:,2)= landmark_map2(:,2);%-73;%+0.9
%-----------------------------------------------------------------%
%plot(estimator.landmark_map(:,1),estimator.landmark_map(:,2),'r.')
%hold on
plot(landmark_map2(:,1),landmark_map2(:,2),'g.')                %<<<first chunck
hold on
axis equal
%-----------------------------------------------------------------%


%=======for getting the clusters, load the lidar sensor readings first
%create a temproary container for lidar readings:
tmp_lidar_reading = [];

for i = 1:size(FG.lidar,2)
    if isempty(FG.lidar{i})
        continue;
    end
    pose=FG.pose{i};
    z=FG.lidar{i};
    b=0;
    a=[cos(pose(9)-b),-sin(pose(9)-b);sin(pose(9)-b),cos(pose(9)-b)]*(z');%*c;
    poseXY=[pose(1);pose(2)];
    a(1,:)=a(1,:)+poseXY(1);
    a(2,:)=a(2,:)+poseXY(2);
    tmp_lidar_reading = [tmp_lidar_reading;a'];
end
plot(tmp_lidar_reading(:,1),tmp_lidar_reading(:,2),'b.')                %<<<second chunck
hold on

%now find the location of three pair of coordinates

lidar_coord =   [ -0.4648,-29.57,0; 14.08,-53.11,0; 18.94,-84.58,0;  2.127,-73.61,0;...
                  25.09,-369,0; 12.45,-384.6,0; 5.435,-218.7,0; 27.62,-390.7,0];
raw_map_coord = [ 1.501,-30.98,0; -12.88,-56.17,0; -17.01,-89.45,0; 0.1673,-77.39,0;...
                 -15.37,-380.4,0; -1.613,-396.4,0; 1.223,-224.3,0; -17.32,-402.8,0];

%find translation and rotation matrix
[R_td,t_td]     = rigid_transform_3D(raw_map_coord, lidar_coord); 
%now let's transform
landmark_map = (R_td*[landmark_map2,zeros(size(landmark_map2,1),1)]')'+t_td';
landmark_map = landmark_map(:,1:2);
landmark_map(:,2) = landmark_map(:,2)-0.6;%y compensation






figure()
hold on
plot(landmark_map(:,1),landmark_map(:,2),'g.')               
hold on
plot(tmp_lidar_reading(:,1),tmp_lidar_reading(:,2),'r.')               
hold on
axis equal

save('landmark_map_old.mat','landmark_map');
%============================dirty math================================
function [R,t] = rigid_transform_3D(A, B)
  centroid_A = mean(A);
  centroid_B = mean(B);
  N = size(A,1);
  H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));
  A_move=A - repmat(centroid_A, N, 1);
  B_move=B - repmat(centroid_B, N, 1);
  A_norm=sum(A_move.*A_move,2);
  B_norm=sum(B_move.*B_move,2);
  lam2=A_norm./B_norm;
  lam2=mean(lam2);

  [U,S,V] = svd(H);

  R = V*U';
  if det(R) < 0
  fprintf('Reflection detected\n');
  V(:,3) = -1*V(:,3);
  R = V*U';
  end
  t = -R./(lam2^(0.5))*centroid_A' + centroid_B';
  R = R./(lam2^(0.5));
  detr=det(R);
end


%-----------------------shit yard--------------------------------------
% ang_deg_roll = -2.25; %0 ;.-0.25
% ang_deg_yaw= 90;%-1.1;%-1.5; %-1.67;
% ang_deg_pitch= -11.5;%-1.5 ; %167.5; %-12; -11.25
% rot_offset_pitch= [cos(ang_deg_pitch*pi/180),0;0,1];
% rot_offset_roll= [1,0;0,cos(ang_deg_roll*pi/180);];
% rot_offset_yaw= [cos(ang_deg_yaw*pi/180),sin(ang_deg_yaw*pi/180);-sin(ang_deg_yaw*pi/180),cos(ang_deg_yaw*pi/180)];
% n_LM= (rot_offset_pitch*rot_offset_roll*rot_offset_yaw*n_LM')';
% 
% n_LM(:,1)= n_LM(:,1)-8;%+6;%0;%+1.1;-8;
% n_LM(:,2)= n_LM(:,2);%-73;%-1.2;-2.0;
% 
% %2D rotation
% trans_M = [0 -1;1 0];
% n_LM = trans_M*n_LM';
% n_LM = n_LM';
% 
% landmark_map = n_LM;
% landmark_map(:,1)= landmark_map(:,1)+0.5;%+6;%0
% landmark_map(:,2)= landmark_map(:,2)+7.5;%-73;%+0.9


%=====================debug_purpose======================03/14/2020======
%load('landmark_map.mat');
%landmark_map = [landmark_map;n_LM];
%scatter(landmark_map(:,1),landmark_map(:,2),10,"blue", 'filled')
%hold on
%scatter(n_LM(:,1),n_LM(:,2),10,"red", 'filled')
    %---------------------------TRAJECTORY----
% filename= 'position.txt';
% file= dlmread(filename);
% scatter(file(:,1),file(:,2),1,'red',"filled")
% axis equal
% hold on
% 
% scatter(landmark_map(:,1),landmark_map(:,2),2,'blue','filled')
% axis equal
%====================================================End_debug============
%save('landmark_map_old.mat','landmark_map');
