filename= 'n_LMmap.txt';
file= csvimport(filename);
n_LM= cell2mat(file(2:end,:));
n_LM(:,2)= n_LM(:,2);
n_LM = [n_LM(:,1),n_LM(:,2)];

ang_deg_roll = 0;
ang_deg_yaw= -1.67%-1.1;%-1.5
ang_deg_pitch= -180.5;%-1.5
rot_offset_pitch= [cos(ang_deg_pitch*pi/180),0;0,1];
rot_offset_roll= [1,0;0,cos(ang_deg_roll*pi/180);];
rot_offset_yaw= [cos(ang_deg_yaw*pi/180),sin(ang_deg_yaw*pi/180);-sin(ang_deg_yaw*pi/180),cos(ang_deg_yaw*pi/180)];
n_LM= (rot_offset_pitch*rot_offset_roll*rot_offset_yaw*n_LM')';

n_LM(:,1)= n_LM(:,1)+1.1;%+6;%0
n_LM(:,2)= n_LM(:,2)-1.2;%-73;

landmark_map = n_LM;
landmark_map(:,1)= landmark_map(:,1)-1.1;%+6;%0
landmark_map(:,2)= landmark_map(:,2)+0.9;%-73;
%load('landmark_map.mat');
%landmark_map = [landmark_map;n_LM];
%scatter(landmark_map(:,1),landmark_map(:,2),10,"blue", 'filled')
%hold on
%scatter(n_LM(:,1),n_LM(:,2),10,"red", 'filled')
save('landmark_map_old.mat','landmark_map');