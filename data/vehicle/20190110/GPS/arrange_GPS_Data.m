%  outputs a matlab variable from the CSV file from ROS
% INPUTS: filename_csv: name of the CSV file
% OUTPUTS: data: matlab variable with the GPS data

filename= 'GPS_message.csv';
data_csv= csvimport(filename);

file_size= size(data_csv);
data= inf*ones(file_size(1)-1,16);
data(:,1)=     cell2mat(data_csv(2:end,28))+2000; % this column is the year (reports 18)
data(:,2)=     cell2mat(data_csv(2:end,27));
data(:,3)=     cell2mat(data_csv(2:end,26));
data(:,4)=     cell2mat(data_csv(2:end,18));
data(:,5:7)=   cell2mat(data_csv(2:end,4:6));
data(:,8:10)=  cell2mat(data_csv(2:end,12:14));
data(:,11:13)= cell2mat(data_csv(2:end,7:9));
data(:,14:16)= cell2mat(data_csv(2:end,15:17));

