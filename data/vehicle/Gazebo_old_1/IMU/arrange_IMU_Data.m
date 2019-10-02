% function [data]=arrange_IMU_Data(filename,test_year,test_month,test_day)

filename= 'STIM_message.csv';
test_year= 2019;
test_month= 09;
test_day= 27;

Data = csvimport(filename);
FileSize=size(Data);
data=inf*ones(FileSize(1)-1,10);
data(:,1)=test_year;
data(:,2)=test_month;
data(:,3)=test_day;
data(:,4)=cell2mat(Data(2:end,1));
data(:,5:10)=cell2mat(Data(2:end,2:7));
save('IMU.mat','data');