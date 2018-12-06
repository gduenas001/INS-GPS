
clear; close all; clc;

global DATA XX PX

addpath('../utils/')

% create objects
params= ParametersClass();
gps= GPSClass(params.numEpochStatic * params.dT_IMU, params);
lidar= LidarClass(params, gps.timeInit);

% ---------------- Read data ----------------
[T_IMU,u,iu]= DataReadIMU(params.fileIMU, gps.timeInit);
N_IMU= size(u,2); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
% -------------------------------------------



% ------------ Initial calibration ------------
load(params.file_name_calibration); % loads iinvC, invC, ib_0, b_0
invC= [invC, zeros(3); zeros(3), eye(3)];

% Initial rotation to get: x=foward & z=down
R_init= [ 0, -1, 0;
         -1, 0, 0;
          0, 0, -1];
R_init_block= blkdiag(R_init,R_init);
iu= (iinvC * iu) - ib_0;
u= (invC * u) - b_0;
u= R_init_block * u;
iu= R_init * iu;
% -------------------------------------------

% ------------ Initial attitude ------------
[phi0, theta0]= initial_attitude( u(1:3, params.numEpochInclCalibration) );
yaw0= deg2rad(-90); % default(180) --Osama-- set the initial yaw angle manually -90
% -------------------------------------------

% Allocate variables
DATA.pred.XX= zeros(15,N_IMU);
DATA.pred.time= zeros(N_IMU,1);
DATA.update.XX= zeros(15,N_IMU);
DATA.update.PX= zeros(15,N_IMU);
DATA.update.time= zeros(N_IMU,1);
LM= [];

% Initialize estimate
XX= zeros(15,1);
PX= zeros(15); 
% PX(7:9,7:9)= diag( [sig_E,sig_E,sig_E] ).^2;
PX(10:12,10:12)= diag( [params.sig_ba,params.sig_ba,params.sig_ba] ).^2;
PX(13:15,13:15)= diag( [params.sig_bw,params.sig_bw,params.sig_bw] ).^2;
XX(7)= phi0;
XX(8)= theta0;
XX(9)= yaw0;
appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong

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
if params.SWITCH_NUM_of_LOOPS ==1
    N_IMU = 46500;
end




