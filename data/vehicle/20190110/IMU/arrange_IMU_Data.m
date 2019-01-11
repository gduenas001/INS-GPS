% function [data]=arrange_IMU_Data(filename,test_year,test_month,test_day)

filename= 'STIM_message.csv';
test_year= 2019;
test_month= 01;
test_day= 10;

Data = csvimport(filename);
FileSize=size(Data);
data=inf*ones(FileSize(1)-1,18);
data(:,1)=test_year;
data(:,2)=test_month;
data(:,3)=test_day;
Dummy_Var=cell2mat(Data(2:end,1))/(1000000000*60*60*24);
data(:,4)=(Dummy_Var-fix(Dummy_Var))*60*60*24;
data(:,5:18)=cell2mat(Data(2:end,2:15));