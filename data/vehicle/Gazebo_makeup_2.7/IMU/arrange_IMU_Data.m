% function [data]=arrange_IMU_Data(filename,test_year,test_month,test_day)

filename= 'STIM_message.csv';
base_time = 878860800;
test_year= 1997;
test_month= 11;
test_day= 7;

Data = csvimport(filename);
FileSize=size(Data);
data=inf*ones(FileSize(1)-1,10);
data(:,1)=test_year;
data(:,2)=test_month;
data(:,3)=test_day;
data(:,4)=cell2mat(Data(2:end,1));
data(:,4)=data(:,4)/1e9+base_time;
data(:,5:10)=cell2mat(Data(2:end,2:7));

%data(:,4) = data(:,4)/1e9
save('IMU.mat','data');