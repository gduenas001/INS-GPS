filename= 'wd_coord2.txt';
file= dlmread(filename);
landmark_map = [file(:,1),file(:,2)];
save('landmark_map_old.mat','landmark_map');