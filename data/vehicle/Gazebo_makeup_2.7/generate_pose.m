%create_pose.txt

filename= load('TIME/corrected_time.mat');
T_LIDAR= filename.time;
T_LIDAR= T_LIDAR(2:end,:);

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
save("pose.mat","new");
%dlmwrite('pose.txt',new);