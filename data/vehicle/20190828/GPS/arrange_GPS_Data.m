%  outputs a matlab variable from the CSV file from ROS
% INPUTS: filename_csv: name of the CSV file
% OUTPUTS: data: matlab variable with the GPS data

filename= 'GPS_message.csv';
data_csv= csvimport(filename);

file_size= size(data_csv);
data= inf*ones(file_size(1)-1,16);
data(:,1)=     2019;
data(:,2)=     8;
data(:,3)=     28;
data(:,4)=     (0:(file_size(1)-2))+fix((cell2mat(data_csv(2,1))/((1e9)*24*60*60)-fix(cell2mat(data_csv(2,1))/((1e9)*24*60*60)))*24*60*60);
%data(:,4)=     cell2mat(data_csv(2:end,1)/((1e18)*24*60*60));
data(:,5:7)=   cell2mat(data_csv(2:end,14:16));
data(:,8:10)=  cell2mat(data_csv(2:end,22:24));
data(:,11:13)= cell2mat(data_csv(2:end,17:19));
data(:,14:16)= cell2mat(data_csv(2:end,25:27));