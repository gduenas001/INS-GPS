filename= 'STIM_message.csv';

% Test date
test_year= 2019;
test_month= 01;
test_day= 10;

% Read CSV file
Data = csvimport(filename);
FileSize=size(Data);

% Define MAT data format
data=inf*ones(FileSize(1)-1,18);

data(:,1)=test_year;
data(:,2)=test_month;
data(:,3)=test_day;

% Convert the PC clock into Seconds from the begining of the day.
Dummy_Var=cell2mat(Data(2:end,1))/(1000000000*60*60*24);
data(:,4)=(Dummy_Var-fix(Dummy_Var))*60*60*24;

% Keep the rest of the data format the same as in the STIM_message_structure.txt file.
data(:,5:18)=cell2mat(Data(2:end,2:15));

% convert Gryo data into rad/sec
gyroX   = deg2rad( data(:,5) );
gyroY   = deg2rad( data(:,6) );
gyroZ   = deg2rad( data(:,7) );
gyroSts = data(:,8);

g0= 9.80665; % m/s^2

% Convert Accelerometer data into m/sec^2 
accX    = data(:,9)  *g0;
accY    = data(:,10) *g0;
accZ    = data(:,11) *g0;
accSts  = data(:,12);

% Convert Inclinometer data into m/sec^2 
incX    = data(:,13) *g0;
incY    = data(:,14) *g0;
incZ    = data(:,15) *g0;
incSts  = data(:,16);
            
% Inclinometer measurment vector
inc_msmt= [incX, incY, incZ]';

% Accelerometer/Gryo measurment vector
msmt= [accX, accY, accZ, gyroX, gyroY, gyroZ]';
            
% Number of measurments
num_readings= size(msmt,2);

% ------------ Static pre-calibration ---------
load('calibration.mat');

% loads iinvC, invC, ib_0, b_0
invC= [invC, zeros(3); zeros(3), eye(3)];

inc_msmt= (iinvC * inc_msmt) - ib_0;
msmt= (invC * msmt) - b_0;
% -------------------------------------------