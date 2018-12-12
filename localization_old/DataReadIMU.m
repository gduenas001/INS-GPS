function [ T, u, iu ] = DataReadIMU( fileName, timeInit )
%DATAREAD Reads a datafile and outputs a series of vectors of the data
% Data id in columns

% load(fileName);

data= importdata(fileName); data= data.data;
N= size(data,1);

g0= 9.80665; % m/s^2 -- by manufacturerw

gyroX= deg2rad(data(:,2));
gyroY= deg2rad(data(:,3));
gyroZ= deg2rad(data(:,4));
accX= data(:,6) *g0;
accY= data(:,7) *g0;
accZ= data(:,8) *g0;
incX= data(:,10) *g0;
incY= data(:,11) *g0;
incZ= data(:,12) *g0;
counterIMU= data(:,15);
T= cumsum( counterIMU*16 / 1e6 );


% % T       = data(:,4);
% % gyroX   = deg2rad( data(:,5) ); 
% % gyroY   = deg2rad( data(:,6) );
% % gyroZ   = deg2rad( data(:,7) );
% % % gyroSts = data(:,8);
% % accX    = data(:,9)  *g0;
% % accY    = data(:,10) *g0;
% % accZ    = data(:,11) *g0;
% % % accSts  = data(:,12);
% % incX    = data(:,13) *g0;
% % incY    = data(:,14) *g0;
% % incZ    = data(:,15) *g0;
% % % incSts  = data(:,16);


% % Use the first GPS reading time as reference
% T= T - timeInit;

% Set paramters
iu= [incX, incY, incZ]';
u= [accX, accY, accZ, gyroX, gyroY, gyroZ]';


end

