function convert_T_LIDAR_from_txt_to_mat()
fileID = fopen('velodyne_frames_timestamps.txt','r');
T_LIDAR = transpose(fscanf(fileID,'%f',[2,inf]));
timestamps_in_days=T_LIDAR(:,2)/(60*60*24);
T_LIDAR(:,2) = (timestamps_in_days-fix(timestamps_in_days))*60*60*24;
save('T_LIDAR.mat','T_LIDAR');
