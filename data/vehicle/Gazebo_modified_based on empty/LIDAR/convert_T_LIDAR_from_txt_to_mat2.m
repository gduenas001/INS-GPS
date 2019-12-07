% function convert_T_LIDAR_from_txt_to_mat()
% converts the lidar time to a mat file. This function is only run once
% before running MAIN


% open and read the text file
%fileID= fopen('ouster_frames_timestamps.txt','r');
filename= 'pose.txt';
file= readtable(filename);
for i = 0:size(file)-1
    T_LIDAR(i+1,1) = i;
end
tmp_mat = table2array(file(:,1));
T_LIDAR = [T_LIDAR,tmp_mat];
%T_LIDAR(:,2)= T_LIDAR(:,2)*(1e9);
%T_LIDAR= transpose(fscanf(fileID,'%f',[2,inf]));

% substract the days from the time
%timestamps_in_days= T_LIDAR(:,2) / (60*60*24);
%T_LIDAR(:,2)= ( timestamps_in_days - fix(timestamps_in_days) ) *60*60*24;

% Add half the difference to the next scan to obtain the time at the middle
% of the scan (the one loaded directly is the time at the beginning of the
% scan)
%diff_T= diff(T_LIDAR(:,2)) / 2;
%diff_T= [diff_T; mean(diff_T)];
%T_LIDAR(:,2)= T_LIDAR(:,2) + diff_T;

% save variable
save('T_LIDAR.mat','T_LIDAR');
