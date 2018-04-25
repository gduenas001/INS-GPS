
clear; format short;% clc; close all;

% Switches (options)
SWITCH_CALIBRATION= 1; % initial calibration to obtain moving biases
SWITCH_VIRT_UPDATE_Z= 0; % virtual update for the z-vel in the body frame
SWITCH_VIRT_UPDATE_Y= 0; % virtual update for the y-vel in the body frame
SWITCH_YAW_UPDATE= 1;
SWITCH_GPS_UPDATE= 1; % update of the GPS
SWITCH_LIDAR_UPDATE= 1;


% Parameters
dT_IMU= 1/125; % IMU sampling time
dT_cal= 1/10; % KF Update period during initial calibration
dT_virt_Z= 1/10; % Virtual msmt update period
dT_virt_Y= 1/10; % Virtual msmt update period
numEpochStatic= 10000; % Number of epochs the cart is static initially
numEpochInclCalibration= round(numEpochStatic);
sig_cal_pos= 0.03; % 3cm   -- do not reduce too much or bias get instable
sig_cal_vel= 0.03; % 3cm/s -- do not reduce too much or bias get instable
sig_cal_E= deg2rad(0.1); % 0.1 deg
sig_E= deg2rad(5); % 5 deg -- Initial uncertatinty in attitude
sig_ba= 0.1; % m/s2 -- Initial acc bias uncertainty
sig_bw= deg2rad(0.2); % 0.1 deg -- Initial gyros bias uncertainty
sig_virt_vz= 0.01; % 5cm/s -- virtual msmt SD in z
sig_virt_vy= 0.01; % 5cm/s -- virtual msmt SD in y
sig_yaw= deg2rad(0.5);
minVelocity= 2/3.6; % 2 km/h
taua0= 6000; % Tau for acc bias -- from manufacturer
tauw0= 6000; % Tau for gyro bias -- from manufacturer
taua_calibration= 80; % 200 acc tau value during initial calibration
tauw_calibration= 80; % 200 gyro tau value during initial calibration
g_val= 9.80279; % value of g [m/s2] at the IIT
r_IMU2rearAxis= 0.9; % distance from IMU to rear axis

% --------------- Initial biases ---------------
load('../calibration/calibration.mat');
invC= [invC, zeros(3); zeros(3), eye(3)];
% ---------------------------------------------

% G estimation (sense is same at grav acceleration in nav-frame)
g_N= [0; 0; g_val];

% Build parameters
sig_cal_pos_blkMAtrix= diag([sig_cal_pos, sig_cal_pos, sig_cal_pos]);
sig_cal_vel_blkMAtrix= diag([sig_cal_vel, sig_cal_vel, sig_cal_vel]);
sig_cal_E_blkMAtrix= diag([sig_cal_E, sig_cal_E, sig_cal_E]);
R_cal= blkdiag(sig_cal_pos_blkMAtrix, sig_cal_vel_blkMAtrix, sig_cal_E_blkMAtrix).^2;
H_cal= [eye(9), zeros(9,6)]; % Calibration observation matrix
H_GPS= [eye(6), zeros(6,9)]; % GPS observation matrix
% H_GPS= [eye(3), zeros(3,12)]; % GPS observation matrix - no velocity
H_yaw= [zeros(1,8),1,zeros(1,6)];
R_virt_Z= sig_virt_vz.^2;
R_virt_Y= sig_virt_vy.^2;
R_yaw= sig_yaw^2;

% IMU -- white noise specs
VRW= 0.07; 
sig_IMU_acc= VRW * sqrt( 2000 / 3600 );
ARW= 0.15; % deg
sig_IMU_gyr= deg2rad( ARW * sqrt( 2000 / 3600 ) ); % rad
V= diag([sig_IMU_acc; sig_IMU_acc; sig_IMU_acc; sig_IMU_gyr; sig_IMU_gyr; sig_IMU_gyr]).^2;
Sv= V * dT_IMU; % Convert to PSD

% Biases -- PSD of white noise
sn_f= ( 0.05 * 9.80279 / 1000 )^2; Sn_f= diag([sn_f, sn_f, sn_f]);
sn_w= ( deg2rad(0.3/3600) )^2;     Sn_w= diag([sn_w, sn_w, sn_w]);
Sn= blkdiag(Sn_f, Sn_w);

% PSD for continuous model
S= blkdiag(Sv, Sn);

% ---------------- Read data ----------------
fileIMU= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/IMU/IMU.mat');
fileGPS= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/GPS/GPS.mat');
fileLIDAR= strcat('../DATA/DATA_COMPLETE/20180419/Smooth_turn/LIDAR/');

% fileIMU= strcat('../DATA/DATA_COMPLETE/20180419/Sharp_turn/IMU/IMU.mat');
% fileGPS= strcat('../DATA/DATA_COMPLETE/20180419/Sharp_turn/GPS/GPS.mat');

[T_GPS,z_GPS,R_GPS,R_NE,timeInit]= dataReadGPS(fileGPS,numEpochStatic*dT_IMU);
[T_IMU,u,iu]= DataReadIMU(fileIMU, timeInit);
T_LIDAR= dataReadLIDARtime(strcat(fileLIDAR,'T_LIDAR.mat'), timeInit);
% -------------------------------------------

% Number of readings
N_IMU= 13000; % size(u,2);
N_GPS= size(z_GPS,2);

% Initial rotation to get: x=foward & z=down
R_init= [ 0, -1, 0;
         -1, 0, 0;
          0, 0, -1];
R_init_block= blkdiag(R_init,R_init);

% calibrate with constant bias and scale factor
iu= (iinvC * iu) - ib_0;
u= (invC * u) - b_0;
u= R_init_block * u;
iu= R_init * iu;

% Initial attitude from inclinometers
[phi0, theta0]= initial_attitude(u(1:3,numEpochInclCalibration));

% Allocate variables
P_store= zeros(15, N_IMU);
P_store_time= zeros(1,N_IMU);
x= zeros(15,N_IMU);
LM= [];

% Initialize estimate
P= zeros(15); 
% P(7:9,7:9)= diag( [sig_E,sig_E,sig_E] ).^2;
P(10:12,10:12)= diag( [sig_ba,sig_ba,sig_ba] ).^2;
P(13:15,13:15)= diag( [sig_bw,sig_bw,sig_bw] ).^2;
x(7,1)= phi0;
x(8,1)= theta0;
yaw0= deg2rad(180); % set the initial yaw angle manually
x(9,1)= yaw0;

% Initialize loop variables
timeSim= 0;
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

% Compute the F and G matrices (linear continuous time)
[F,G]= FG_fn(u(1,1),u(2,1),u(3,1),u(5,1),u(6,1),...
             x(7,1),x(8,1),x(9,1),x(10,1),x(11,1),x(12,1),x(14,1),x(15,1),taua,tauw);

% Discretize system for IMU (only for variance calculations)
[Phi,D_bar]= discretize(F, G, H_cal, S, dT_IMU);

% --------------------- LOOP ---------------------
for k= 1:N_IMU-1
    
    % Turn off GPS updates if start moving
    if k == numEpochStatic
        SWITCH_CALIBRATION= 0; 
        P(9,9)= sig_E^2;
        taua= taua0;
        tauw= tauw0;
    end
        
    % Increase time count
    timeSim= timeSim + dT_IMU;
    timeSum= timeSum + dT_IMU;
    timeSumVirt_Z= timeSumVirt_Z + dT_IMU;
    timeSumVirt_Y= timeSumVirt_Y + dT_IMU;
    
    % Update position mean
    x(:,k+1)= IMU_update( x(:,k), u(:,k), g_N, taua, tauw, dT_IMU );
    
    % Update cov matrix
    P= Phi*P*Phi' + D_bar;
    
    % KF update
    if timeSum >= dT_cal && SWITCH_CALIBRATION
        
        % Compute the F and G matrices (linear continuous time)
        [F,G]= FG_fn(u(1,k),u(2,k),u(3,k),u(5,k),u(6,k),...
            x(7,k+1),x(8,k+1),x(9,k+1),x(10,k+1),x(11,k+1),x(12,k+1),x(14,k+1),x(15,k+1),taua,tauw);
        
        % Discretize system for IMU time (only for variance calculations)
        [Phi,D_bar]= discretize(F, G, H_cal, S, dT_IMU);
        
        % Calibration msmt update
        L= P*H_cal' / (H_cal*P*H_cal' + R_cal);
        z= [zeros(6,1); phi0; theta0; yaw0];
        z_hat= H_cal*x(:,k+1);
        innov= z - z_hat;
        innov(end)= pi_to_pi(innov(end));
        x(:,k+1)= x(:,k+1) + L*innov;
        P= P - L*H_cal*P;
        
        % If GPS is calibrating initial biases, increse bias variance
        D_bar(10:12,10:12)= D_bar(10:12,10:12) + diag( [sig_ba,sig_ba,sig_ba] ).^2;
        D_bar(13:15,13:15)= D_bar(13:15,13:15) + diag( [sig_bw,sig_bw,sig_bw] ).^2;
        
        % Store cov matrix
        P_store(:,k_update) = diag(P);
        P_store_time(k_update)= timeSim;
        
        % Time counters
        timeSum= 0;
        k_update= k_update+1;
    end
    % Vehicle moving - no calibration and virtual msmts    
    if timeSumVirt_Z >= dT_virt_Z && SWITCH_VIRT_UPDATE_Z && ~SWITCH_CALIBRATION        
        % Virtual msmt update -- Z vel
        R_BN= R_NB_rot( x(7,k+1), x(8,k+1), x(9,k+1) )';
        H_virt= H_fn(x(4,k+1), x(5,k+1), x(6,k+1), x(7,k+1), x(8,k+1), x(9,k+1));
        L= P*H_virt' / (H_virt*P*H_virt' + R_virt_Z);
        z= 0;
        z_hat= [0,0,1] * R_BN * x(4:6,k+1);
        innov= z - z_hat;
        x(:,k+1)= x(:,k+1) + L*innov;
        P= P - L*H_virt*P;
                
        % Time counter
        timeSumVirt_Z= 0;
    end
    % Vehicle moving - no calibration and virtual msmts    
    if timeSumVirt_Y >= dT_virt_Y && SWITCH_VIRT_UPDATE_Y && ~SWITCH_CALIBRATION        
        % Virtual msmt update -- Y vel in axis
        R_BN= R_NB_rot( x(7,k+1), x(8,k+1), x(9,k+1) )';
        H_virt= [0, 1, 0] * R_BN * [zeros(3), eye(3), zeros(3,9)];
        L= P*H_virt' / (H_virt*P*H_virt' + R_virt_Y);
        z= r_IMU2rearAxis * u(6,k);
        z_hat= H_virt * x(:,k+1);
        innov= z - z_hat;
        x(:,k+1)= x(:,k+1) + L*innov;
        
        % Time counter
        timeSumVirt_Y= 0;
    end
    % Vehicle moving - GPS update
    if (timeSim + dT_IMU) > timeGPS && SWITCH_GPS_UPDATE 
        
        if ~SWITCH_CALIBRATION
            
            % GPS msmt update
            R= diag( R_GPS(:,k_GPS) );
%             R= diag( R_GPS(1:3,k_GPS) );
            L= P*H_GPS' / (H_GPS*P*H_GPS' + R);
            innov= z_GPS(:,k_GPS) - H_GPS*x(:,k+1);
%             innov= z_GPS(1:3,k_GPS) - H_GPS*x(:,k+1);
            x(9,k+1)= pi_to_pi(x(9,k+1));
            x(:,k+1)= x(:,k+1) + L*innov;
            P= P - L*H_GPS*P;
            
            % Yaw update 
            if SWITCH_YAW_UPDATE && norm(x(4:6,k+1)) > minVelocity
                z= yawUpdate(x(:,k+1),u(4:6,k),r_IMU2rearAxis);
                L= P*H_yaw' / (H_yaw*P*H_yaw' + R_yaw);
                innov= z - H_yaw*x(:,k+1);
                innov= pi_to_pi(innov);
                x(9,k+1)= pi_to_pi(x(9,k+1));
                x(:,k+1)= x(:,k+1) + L*innov;
                P= P - L*H_yaw*P;
            end
            
            % Compute the F and G matrices (linear continuous time)
            [F,G]= FG_fn(u(1,k),u(2,k),u(3,k),u(5,k),u(6,k),...
                x(7,k+1),x(8,k+1),x(9,k+1),x(10,k+1),x(11,k+1),x(12,k+1),x(14,k+1),x(15,k+1),taua,tauw);
            
            % Discretize system for IMU time (only for variance calculations)
            [Phi,D_bar]= discretize(F, G, H_cal, S, dT_IMU);
                       
            % Store cov matrix
            P_store(:,k_update)= diag(P);
            P_store_time(k_update)= timeSim;
            k_update= k_update+1;
            
        end
        
        % Time GPS counter
        k_GPS= k_GPS + 1;
        timeGPS= T_GPS(k_GPS);
    end
    
    % LIDAR 
    if (timeSim + dT_IMU) > timeLIDAR && SWITCH_LIDAR_UPDATE
        epochLIDAR= T_LIDAR(k_LIDAR,1);
        
        % Read the lidar features
        z_LIDAR= dataReadLIDAR(fileLIDAR, epochLIDAR);
        
        R_NB= R_NB_rot(x(7,k+1),x(8,k+1),x(9,k+1));
        z_LIDAR= ( R_NB * z_LIDAR' )';
        z_LIDAR(:,1)= z_LIDAR(:,1) + x(1,k+1);
        z_LIDAR(:,2)= z_LIDAR(:,2) + x(2,k+1);
        z_LIDAR(:,3)= z_LIDAR(:,3) + x(3,k+1);
        
        LM= [LM; z_LIDAR];
        
        k_LIDAR= k_LIDAR + 1;
        timeLIDAR= T_LIDAR(k_LIDAR,2);
        
    end
    
end
% Store final variance
P_store(:, k_update)= diag(P);
P_store_time(k_update)= timeSim;
% --------------------- END LOOP ---------------------

numEpochInitPlot= numEpochStatic;

timeComplete= 0:dT_IMU:timeSim+dT_IMU/2;
timeMove= timeComplete(numEpochInitPlot:end);

% Plot estimates
figure; hold on;

subplot(3,3,1); hold on; grid on;
plot(timeComplete,x(1,:));
ylabel('x [m]');

subplot(3,3,2); hold on; grid on;
plot(timeComplete,x(2,:));
ylabel('y [m]');

subplot(3,3,3); hold on; grid on; 
plot(timeComplete,x(3,:))
ylabel('z [m]');

subplot(3,3,4); hold on; grid on;
plot(timeComplete,x(4,:))
ylabel('v_x [m/s]');

subplot(3,3,5); hold on; grid on;
plot(timeComplete,x(5,:))
ylabel('v_y [m/s]');

subplot(3,3,6); hold on; grid on;
plot(timeComplete,x(6,:));
ylabel('v_z [m/s]');

subplot(3,3,7); hold on; grid on;
plot(timeComplete,rad2deg(x(7,:)))
ylabel('\phi [deg]');

subplot(3,3,8); hold on; grid on;
plot(timeComplete,rad2deg(x(8,:)))
ylabel('\theta [deg]');

subplot(3,3,9); hold on; grid on;
plot(timeComplete,rad2deg(x(9,:)))
ylabel('\psi [deg]');

% % Plot biases
% figure; hold on; grid on; title('biases in accelerometers');
% plot(timeComplete, x(10,:), 'linewidth',2)
% plot(timeComplete, x(11,:), 'linewidth',2)
% plot(timeComplete, x(12,:), 'linewidth',2)
% ylabel('m/s^2')
% legend('x','y','z')
% 
% figure; hold on; grid on; title('biases in gyros');
% plot(timeComplete, rad2deg(x(13,:)), 'linewidth',2)
% plot(timeComplete, rad2deg(x(14,:)), 'linewidth',2)
% plot(timeComplete, rad2deg(x(15,:)), 'linewidth',2)
% ylabel('deg');
% legend('w_x','w_y','w_z')

% Plot GPS+IMU estimated path
figPath= figure; hold on; grid on;
plot3(x(1,:),x(2,:),x(3,:),'b.');
plot3(z_GPS(1,:),z_GPS(2,:),z_GPS(3,:),'r*');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
plot_attitude( x(1:9,1:300:end), figPath ) % Plot attitude at some epochs
axis equal

% % Plot GPS positions
% figure; hold on; grid on; title('GPS positions');
% plot(1:N_GPS, z_GPS(1,:), 'linewidth',2)
% plot(1:N_GPS, z_GPS(2,:), 'linewidth',2)
% plot(1:N_GPS, z_GPS(3,:), 'linewidth',2)
% ylabel('m')
% legend('r_x','r_y','r_z')

% % Plot GPS velocities
% figure; hold on; grid on; title('GPS velocities');
% plot(1:N_GPS, z_GPS(4,:), 'linewidth',2)
% plot(1:N_GPS, z_GPS(5,:), 'linewidth',2)
% plot(1:N_GPS, z_GPS(6,:), 'linewidth',2)
% ylabel('m/s')
% legend('v_x','v_y','v_z')


% % Plot measurements
% figure; hold on; grid on;
% u_ax_filter= filter(ones(1,500)/500,1,u(1,:));
% u_ay_filter= filter(ones(1,500)/500,1,u(2,:));
% plot(timeComplete, u_ax_filter(:));
% plot(timeComplete, u_ay_filter(:));
% legend('accX','accY')


% % Plot variance estimates
% SD= sqrt(P_store(:,1:k_update));
% P_store_time= P_store_time(1:k_update);
% 
% figure; hold on; grid on;
% plot(P_store_time, SD(1,:),'b-','linewidth',2);
% plot(P_store_time, SD(2,:),'r-','linewidth',2);
% plot(P_store_time, SD(3,:),'g-','linewidth',2);
% xlabel('Time epochs')
% ylabel('m');
% legend('x','y','z');

% Plot landmarks
if SWITCH_LIDAR_UPDATE
    figure(figPath); hold on;
    plot3(LM(:,1),LM(:,2),LM(:,3),'k.');
end







