clear; clc; close all;

% Paramters
g= 9.80665;

% Read data
Data= importdata('../DATA_STATIC/1x2M/2M_static.txt','\t',1);

T= Data.data(:,1);
wx= (Data.data(:,2));
wy= (Data.data(:,3));
wz= (Data.data(:,4));
wsts= Data.data(:,5);
ax= Data.data(:,6).*g;
ay= Data.data(:,7).*g;
az= Data.data(:,8).*g;
asts= Data.data(:,9);

if any(asts) || any(wsts)
    error('some corrupt readings');
end

clear Data
%% 

% Parameters from data
N= length(T);
dT= diff(T);

% wx
mu_wx= mean(wx);
sig_wx= sqrt( 1/(N-1) * sum( (wx - mu_wx).^2 ) )

% wy
mu_wy= mean(wy);
sig_wy= sqrt( 1/(N-1) * sum( (wy - mu_wy).^2 ) )

% wz
mu_wz= mean(wz);
sig_wz= sqrt( 1/(N-1) * sum( (wz - mu_wz).^2 ) )

% ax
mu_ax= mean(ax);
sig_ax= sqrt( 1/(N-1) * sum( (ax - mu_ax).^2 ) )

% ay
mu_ay= mean(ay);
sig_ay= sqrt( 1/(N-1) * sum( (ay - mu_ay).^2 ) )

% az
mu_az= mean(az);
sig_az= sqrt( 1/(N-1) * sum( (az - mu_az).^2 ) )


% Angle Random Walk (ARW)
ARW_wx= sig_wx * sqrt(3600/2000)
ARW_wy= sig_wy * sqrt(3600/2000)
ARW_wz= sig_wz * sqrt(3600/2000)

% Position Random Walk (VRW)
VRW_ax= sig_ax * sqrt(3600/2000)
VRW_ay= sig_ay * sqrt(3600/2000)
VRW_az= sig_az * sqrt(3600/2000)




