filename= '224LM.txt';
file= csvimport(filename);
n_LM= cell2mat(file(2:end,:));
n_LM(:,2)= n_LM(:,2);
n_LM = [n_LM(:,1),n_LM(:,2)];

ang_deg_roll = -2.25; %0 ;.-0.25
ang_deg_yaw= 90;%-1.1;%-1.5; %-1.67;
ang_deg_pitch= -11.5;%-1.5 ; %167.5; %-12; -11.25
rot_offset_pitch= [cos(ang_deg_pitch*pi/180),0;0,1];
rot_offset_roll= [1,0;0,cos(ang_deg_roll*pi/180);];
rot_offset_yaw= [cos(ang_deg_yaw*pi/180),sin(ang_deg_yaw*pi/180);-sin(ang_deg_yaw*pi/180),cos(ang_deg_yaw*pi/180)];
n_LM= (rot_offset_pitch*rot_offset_roll*rot_offset_yaw*n_LM')';

n_LM(:,1)= n_LM(:,1)-8;%+6;%0;%+1.1;-8;
n_LM(:,2)= n_LM(:,2);%-73;%-1.2;-2.0;

%2D rotation
trans_M = [0 -1;1 0];
n_LM = trans_M*n_LM';
n_LM = n_LM';

landmark_map = n_LM;
landmark_map(:,1)= landmark_map(:,1)+0.5;%+6;%0
landmark_map(:,2)= landmark_map(:,2)+7.5;%-73;%+0.9
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
save('landmark_map_old.mat','landmark_map');
