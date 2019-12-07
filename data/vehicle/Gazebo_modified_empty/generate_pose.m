%create_pose.txt

filename= 'ouster_frames_timestamps.txt';
file= csvimport(filename);
T_LIDAR= cell2mat(file(2:end,:));
T_LIDAR(:,2)= T_LIDAR(:,2);

filename= 'position.txt';
file= dlmread(filename);

filename2= 'attitude.txt';
file2= dlmread(filename2);
d = zeros(length(file));
d = d(:,1);

file2(:,1) = degtorad(file2(:,1));
file2(:,2) = degtorad(file2(:,2));
file2(:,3) = degtorad(file2(:,3));

%file3(:,1) = file(:,3);
%file3(:,2) = file(:,2);
%file3(:,3) = file(:,1)
p = T_LIDAR(:,2);

%file = [[0,0,0];file];
%file2 = [[0,0,0];file2];
new = [p,file,d,d,d,file2];
dlmwrite('pose.txt',new);