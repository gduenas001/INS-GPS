function [ T, gyroX, gyroY, gyroZ, accX, accY, accZ, incX, incY, incZ,...
    gyroSts, accSts, incSts, counter, latency, g0 ] = DataRead( string )

%DATAREAD Reads a datafile and outputs a series of vectors of the data
% Data id in columns

Data= importdata(string,'\t',1);
g0= 9.80665; % m/s^2

T       = Data(1).data(:,1);
gyroX   = Data(1).data(:,2)/180*pi;
gyroY   = Data(1).data(:,3)/180*pi;
gyroZ   = Data(1).data(:,4)/180*pi;
gyroSts = Data(1).data(:,5);
accX    = Data(1).data(:,6)*g0;
accY    = Data(1).data(:,7)*g0;
accZ    = Data(1).data(:,8)*g0;
accSts  = Data(1).data(:,9);
incX    = Data(1).data(:,10)*g0;
incY    = Data(1).data(:,11)*g0;
incZ    = Data(1).data(:,12)*g0;
incSts  = Data(1).data(:,13);
counter = Data(1).data(:,14);
latency = Data(1).data(:,15);

end

