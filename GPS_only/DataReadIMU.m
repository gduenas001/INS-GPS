function [ T, u, iu ] = DataReadIMU( string )
%DATAREAD Reads a datafile and outputs a series of vectors of the data
% Data id in columns
load(string);

g0= 9.80665; % m/s^2 -- by manufacturerw

T       = data(:,4);
gyroX   = deg2rad( data(:,5) ); 
gyroY   = deg2rad( data(:,6) );
gyroZ   = deg2rad( data(:,7) );
% gyroSts = data(:,8);
accX    = data(:,9)  *g0;
accY    = data(:,10) *g0;
accZ    = data(:,11) *g0;
% accSts  = data(:,12);
incX    = data(:,13) *g0;
incY    = data(:,14) *g0;
incZ    = data(:,15) *g0;
% incSts  = data(:,16);

% start time at zero
T= T - T(1);

% Set paramters
iu= [incX, incY, incZ]';
u= [accX, accY, accZ, gyroX, gyroY, gyroZ]';


end

