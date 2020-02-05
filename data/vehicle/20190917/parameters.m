 % --------------- Switches (options) ---------------
 SWITCH_REDUCE_TESTING= 0; % to test only a few frames
 SWITCH_VIRT_UPDATE_Z= 0; % virtual update for the z-vel in the body frame
 SWITCH_VIRT_UPDATE_Y= 0; % virtual update for the y-vel in the body frame
 SWITCH_YAW_UPDATE= 0;
 SWITCH_GPS_UPDATE= 1; % update of the GPS
 SWITCH_GPS_VEL_UPDATE= 1; % update of the GPS
 SWITCH_LIDAR_UPDATE= 1;
 SWITCH_REMOVE_FAR_FEATURES= 1;
 SWITCH_CALIBRATION= 1; % initial calibration to obtain moving biases
 SWITCH_FIXED_LM_SIZE_PH= 1;
 SWITCH_LM_SELECTION= 0; % landmark selection activation
 SWITCH_SEED= 1; 
 SWITCH_GENERATE_RANDOM_MAP= 0;
 SWITCH_LIDAR_FAULTS= 0;
 SWITCH_ONLY_ONE_LM_FAULT= 1; % only monitor one simultaneous landmark failing
 SWITCH_GPS_FG= 1;
 SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt= 0;
 SWITCH_SM=1;
 % --------------------------------------------------


% --------------- Parameters ---------------
m= 15; % number of states in the state vector
P_MA_max= 1e-4; % maximum alloable miss-association probability
P_UA= 1e-9;
I_H= 1e-12;
min_n_L_M= 30; % min number of landmarks in the preceding horizon + current time
min_appearances= 5; % only update estimate landmarks detected more than this number
num_epochs_reduce_testing= 6000;
num_epochs_static= 10000; % default (10000) --Osama-- Number of epochs the cart is static initially 20000
lidarRange= 25; % [m]
m_F= 2; % measurements per feature/landmark
dt_imu= 1/125; % IMU sampling time
dt_cal= 1/10; % KF Update period during initial calibration
dt_virt_z= 1/10; % Virtual msmt update period
dt_virt_y= 1/10; % Virtual msmt update period
sig_cal_pos= 0.01; % 3cm   -- do not reduce too much or bias get instable
sig_cal_vel= 0.01; % 3cm/s -- do not reduce too much or bias get instable
sig_cal_E= deg2rad(0.1); % 0.1 deg
sig_yaw0= deg2rad(5); % 5 deg -- Initial uncertatinty in yaw
sig_phi0= deg2rad(1); % 2 deg -- Initial uncertatinty in attitude
sig_ba= 0.05; % 0.1 m/s2 -- Initial acc bias uncertainty
sig_bw= deg2rad(0.1); % 0.2 deg/s -- Initial gyros bias uncertainty
sig_virt_vz= 0.01; % 5cm/s -- virtual msmt SD in z
sig_virt_vy= 0.01; % 5cm/s -- virtual msmt SD in y
sig_lidar= 0.3; % 25cm -- lidar measurement in the nav frame
min_vel_gps= 2/3.6; % 2 km/h
min_vel_yaw= 2/3.6; % 2 km/h
taua_normal_operation= 3000; % Tau for acc bias -- from manufacturer (default = 3000)
tauw_normal_operation= 3000; % Tau for gyro bias -- from manufacturer (default = 3000)
taua_calibration= 100; % 200 acc tau value during initial calibration
tauw_calibration= 100; % 200 gyro tau value during initial calibration
g_val= 9.80279; % value of g [m/s2] at the IIT
r_IMU2rearAxis= 0.9; % distance from IMU to rear axis
alpha_NN= 0.05; % prob of discard good features in NN
threshold_new_landmark= 10; % Threshold in NIS to create a new landmark
sig_minLM= 0.2; % minimum SD for the landmarks
mult_factor_acc_imu= 10; % multiplicative factor for the accel SD
mult_factor_gyro_imu= 10; % multiplicative factor for the gyros SD
mult_factor_pose_gps= 1; % multiplicative factor for the GPS pose SD
mult_factor_vel_gps= 1;  % multiplicative factor for the GPS velocity SD
feature_height= 0;%1.5; % height of the features
initial_yaw_angle= -90; % [deg] initial yaw angle (different for each experiment) % smooth_turn(180)
preceding_horizon_size= 5; % size of the preceding horizon in epochs
continuity_requirement= 1e-5;
alert_limit= 1.0;
VRW= 0.07; % vel random walk
ARW= 0.15; % angular random walk [deg]
sn_f= ( 0.05 * 9.80279 / 1000 )^2; % bias acc white noise PSD
sn_w= ( deg2rad(0.3/3600) )^2;    % bias gyro white noise PSD
I_MA= 1e-12;
% -------------------------------------------
