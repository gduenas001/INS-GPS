
global DATA XX PX

% fileIMU= strcat('../DATA/DATA_COMPLETE/20180426/Guillermo/IMU/IMU.mat');
% fileGPS= strcat('../DATA/DATA_COMPLETE/20180426/Guillermo/GPS/GPS.mat');


% fileIMU= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/IMU/20180419_1.txt');
% fileIMU= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/IMU/IMU.mat');
% fileGPS= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/GPS/GPS.mat');
% fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/LIDAR/');

% dataset obtained with ROS
fileIMU= strcat('../DATA/DATA_COMPLETE/20180725/IMU/IMU.mat');
fileGPS= strcat('../DATA/DATA_COMPLETE/20180725/GPS/GPS.mat');
fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180725/LIDAR/');

% fileIMU= strcat('../DATA/DATA_COMPLETE/20180821/IMU/IMU.mat');
% fileGPS= strcat('../DATA/DATA_COMPLETE/20180821/GPS/GPS.mat');
% fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180821/LIDAR/');


% fileIMU= strcat('../DATA/DATA_COMPLETE/20180419/Sharp_turn/IMU/IMU.mat');
% fileGPS= strcat('../DATA/DATA_COMPLETE/20180419/Sharp_turn/GPS/GPS.mat');

% --------------- Initial biases ---------------
load('../calibration/calibration.mat');
invC= [invC, zeros(3); zeros(3), eye(3)];
% ---------------------------------------------


% --------------- Switches (options) ---------------
SWITCH_NUM_of_LOOPS= 1; % --Osama--
SWITCH_CALIBRATION= 1; % initial calibration to obtain moving biases
SWITCH_VIRT_UPDATE_Z= 0; % virtual update for the z-vel in the body frame
SWITCH_VIRT_UPDATE_Y= 0; % virtual update for the y-vel in the body frame
SWITCH_YAW_UPDATE= 1;
SWITCH_GPS_UPDATE= 1; % update of the GPS
SWITCH_GPS_VEL_UPDATE= 1; % update of the GPS
SWITCH_LIDAR_UPDATE= 1;
SWITCH_REMOVE_FAR_FEATURES= 1;
% --------------------------------------------------

% --------------- Parameters ---------------
dT_IMU= 1/125; % IMU sampling time
dT_cal= 1/10; % KF Update period during initial calibration
dT_virt_Z= 1/10; % Virtual msmt update period
dT_virt_Y= 1/10; % Virtual msmt update period
numEpochStatic= 20000; % default (10000) --Osama-- Number of epochs the cart is static initially 20000
numEpochInclCalibration= round(numEpochStatic);
sig_cal_pos= 0.005; % 3cm   -- do not reduce too much or bias get instable
sig_cal_vel= 0.005; % 3cm/s -- do not reduce too much or bias get instable
sig_cal_E= deg2rad(0.1); % 0.1 deg
sig_yaw0= deg2rad(5); % 5 deg -- Initial uncertatinty in attitude
sig_phi0= deg2rad(1); % 2 deg -- Initial uncertatinty in attitude
sig_ba= 0.05; % 0.1 m/s2 -- Initial acc bias uncertainty
sig_bw= deg2rad(0.1); % 0.2 deg/s -- Initial gyros bias uncertainty
sig_virt_vz= 0.01; % 5cm/s -- virtual msmt SD in z
sig_virt_vy= 0.01; % 5cm/s -- virtual msmt SD in y
sig_lidar= 0.3; % 20cm -- lidar measurement in the nav frame
sig_yaw_fn= @(v) deg2rad(5) + ( exp(10*v)-1 )^(-1); %6.6035  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAREFUL
minVelocityGPS= 2/3.6; % 2 km/h
minVelocityYaw= 2/3.6; % 2 km/h
taua0= 3000; % Tau for acc bias -- from manufacturer
tauw0= 3000; % Tau for gyro bias -- from manufacturer
taua_calibration= 100; % 200 acc tau value during initial calibration
tauw_calibration= 100; % 200 gyro tau value during initial calibration
g_val= 9.80279; % value of g [m/s2] at the IIT
r_IMU2rearAxis= 0.9; % distance from IMU to rear axis
lidarRange= 25; % [m]
alpha_NN= 0.05; % prob of discard good features in NN
T_newLM= 15; % Threshold in NIS to create a new landmark
sig_minLM= 0.1; % minimum SD for the landmarks
multFactorAccIMU= 30; % multiplicative factor for the accel SD
multFactorGyroIMU= 30; % multiplicative factor for the gyros SD
multFactorPoseGPS= 3; % multiplicative factor for the GPS pose SD
multFactorVelGPS= 20;  % multiplicative factor for the GPS velocity SD
% -------------------------------------------

% ---------------- Read data ----------------
[T_GPS,z_GPS,R_GPS,R_NE,timeInit]= dataReadGPS(fileGPS,numEpochStatic*dT_IMU);
R_GPS(1:3,:)= R_GPS(1:3,:)*(multFactorPoseGPS^2); %%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAREFUL
R_GPS(4:6,:)= R_GPS(4:6,:)*(multFactorVelGPS^2);  %%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAREFUL
[T_IMU,u,iu]= DataReadIMU(fileIMU, timeInit);    
T_LIDAR= dataReadLIDARtime(strcat(fileLIDAR,'T_LIDAR.mat'), timeInit);
% -------------------------------------------


% --------------- Build parameters ---------------
g_N= [0; 0; g_val]; % G estimation (sense is same at grav acceleration in nav-frame)
sig_cal_pos_blkMAtrix= diag([sig_cal_pos, sig_cal_pos, sig_cal_pos]);
sig_cal_vel_blkMAtrix= diag([sig_cal_vel, sig_cal_vel, sig_cal_vel]);
sig_cal_E_blkMAtrix= diag([sig_cal_E, sig_cal_E, sig_cal_E]);
R_cal= blkdiag(sig_cal_pos_blkMAtrix, sig_cal_vel_blkMAtrix, sig_cal_E_blkMAtrix).^2;
H_cal= [eye(9), zeros(9,6)]; % Calibration observation matrix
H_yaw= [zeros(1,8),1,zeros(1,6)];
R_virt_Z= sig_virt_vz.^2;
R_virt_Y= sig_virt_vy.^2;
R_lidar= diag( [sig_lidar, sig_lidar] ).^2;
R_yaw_fn= @(v) sig_yaw_fn(v)^2; 
T_NN= 4.5; %chi2inv(1-alpha_NN,2);
xPlot= [-0.3; 0; -0.3];
yPlot= [0.1; 0; -0.1];
zPlot= [0; 0; 0];
xyz_B= [xPlot, yPlot, zPlot]';
R_minLM= sig_minLM.^2;
% ----------------------------------------------


% IMU -- white noise specs
VRW= 0.07 *multFactorAccIMU;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
sig_IMU_acc= VRW * sqrt( 2000 / 3600 );
ARW= 0.15 *multFactorGyroIMU; % deg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
sig_IMU_gyr= deg2rad( ARW * sqrt( 2000 / 3600 ) ); % rad
V= diag([sig_IMU_acc; sig_IMU_acc; sig_IMU_acc; sig_IMU_gyr; sig_IMU_gyr; sig_IMU_gyr]).^2;
Sv= V * dT_IMU; % Convert to PSD
Sv_cal= diag( [diag(Sv(1:3,1:3)) / multFactorAccIMU^2; diag(Sv(4:6,4:6)) / multFactorGyroIMU^2]  );

% Biases -- PSD of white noise
sn_f= ( 0.05 * 9.80279 / 1000 )^2; Sn_f= diag([sn_f, sn_f, sn_f]);
sn_w= ( deg2rad(0.3/3600) )^2;     Sn_w= diag([sn_w, sn_w, sn_w]);
Sn= blkdiag(Sn_f, Sn_w);

% PSD for continuous model
S= blkdiag(Sv, Sn);
S_cal= blkdiag(Sv_cal, Sn);

% Number of readings
N_IMU= size(u,2);% N_IMU= 15000; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAREFUL
N_GPS= size(z_GPS,2);

% Initial rotation to get: x=foward & z=down
R_init= [ 0, -1, 0;
         -1, 0, 0;
          0, 0, -1];
R_init_block= blkdiag(R_init,R_init);

% ------------ Initial calibration ------------
iu= (iinvC * iu) - ib_0;
u= (invC * u) - b_0;
u= R_init_block * u;
iu= R_init * iu;
% -------------------------------------------

% ------------ Initial attitude ------------
[phi0, theta0]= initial_attitude(u(1:3,numEpochInclCalibration));
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
PX(10:12,10:12)= diag( [sig_ba,sig_ba,sig_ba] ).^2;
PX(13:15,13:15)= diag( [sig_bw,sig_bw,sig_bw] ).^2;
XX(7)= phi0;
XX(8)= theta0;
XX(9)= yaw0;
appearances= zeros(1,300); % if there are more than 300 landmarks, something's wrong

% Initialize loop variables
timeSum= 0;
timeSumVirt_Z= 0;
timeSumVirt_Y= 0;
timeGPS= T_GPS(1); % this is zero, as the GPS time is the reference
timeLIDAR= T_LIDAR(1,2);
k_update= 1;
k_GPS= 1;
k_LIDAR= 1;
taua= taua_calibration;
tauw= tauw_calibration;
GPS_Index_exceeded = 0;
LIDAR_Index_exceeded = 0;
if SWITCH_NUM_of_LOOPS ==1
    N_IMU = 46500;
end




