
clear; close all; clc;

global DATA 
% global XX PX

addpath('../utils/')

% create objects
params= ParametersClass();
gps= GPSClass(params.numEpochStatic * params.dT_IMU, params);
lidar= LidarClass(params, gps.timeInit);
imu= IMUClass(params, gps.timeInit);
estimator= EstimatorClass(imu, params);


% % Initialize estimate
% [phi0, theta0]= initial_attitude( imu.msmt(1:3, params.numEpochInclCalibration) );
% yaw0= deg2rad(params.initial_yaw_angle); 
% XX= zeros(15,1);
% PX= zeros(15); 
% % PX(7:9,7:9)= diag( [sig_E,sig_E,sig_E] ).^2;
% PX(10:12, 10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
% PX(13:15, 13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
% XX(7)= phi0;
% XX(8)= theta0;
% XX(9)= yaw0;
% appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong

% Allocate variables
DATA.pred.XX= zeros(15, imu.num_readings);
DATA.pred.time= zeros(imu.num_readings, 1);
DATA.update.XX= zeros(15, imu.num_readings);
DATA.update.PX= zeros(15, imu.num_readings);
DATA.update.time= zeros(imu.num_readings, 1);
LM= [];

% Initialize loop variables
timeSum= 0;
timeSumVirt_Z= 0;
timeSumVirt_Y= 0;
timeGPS= gps.time(1); % this is zero, as the GPS time is the reference
timeLIDAR= lidar.time(1,2);
k_update= 1;
k_GPS= 1;
k_LIDAR= 1;
taua= params.taua_calibration;
tauw= params.tauw_calibration;
GPS_Index_exceeded = 0;
LIDAR_Index_exceeded = 0;




