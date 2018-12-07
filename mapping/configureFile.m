
clear; close all; clc;

global DATA 

addpath('../utils/')

% create objects
params= ParametersClass();
gps= GPSClass(params.numEpochStatic * params.dT_IMU, params);
lidar= LidarClass(params, gps.timeInit);
imu= IMUClass(params, gps.timeInit);
estimator= EstimatorClass(imu.msmt(1:3, params.numEpochInclCalibration), params);

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




