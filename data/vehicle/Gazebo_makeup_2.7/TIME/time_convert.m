% a local time to POSIX time converter
% Correct the timestamp.txt
% base time 1997-11-07 00:00:00 in nano second, which is 878860800 in
% second

base_time = 878860800;

filename= 'ouster_frames_timestamps.txt';
file= csvimport(filename);
T= cell2mat(file(2:end,:));
T(:,2) = base_time+T(:,2);
time = [0,0;T];
save("corrected_time.mat","time");

