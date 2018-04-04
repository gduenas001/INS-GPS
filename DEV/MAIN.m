clear; format short; clc; close all;

% Parameters
dT_GNSS= 1; % KF Update period
dT_IMU= 1/2000;

% Observation matrix
H= [eye(3), zeros(3,12)];

% White noise specs
VRW= 0.07; %
sig_IMU_acc= VRW * sqrt( 2000 / 3600 );
ARW= 0.15; % deg
sig_IMU_gyr= deg2rad( ARW * sqrt( 2000 / 2600 ) ); % rad
V= diag([sig_IMU_acc; sig_IMU_acc; sig_IMU_acc; sig_IMU_gyr; sig_IMU_gyr; sig_IMU_gyr]).^2;

% Convert to PSD
Sv= V * dT_IMU;

% Tau for bias -- from manufacturer
tau= 3600;

% PSD for bias random noise
sn_f= ( 0.05 * 9.80279 / 1000 )^2; Sn_f= diag([sn_f, sn_f, sn_f]);
sn_w= ( deg2rad(0.3/3600) )^2;     Sn_w= diag([sn_w, sn_w, sn_w]);
Sn= blkdiag(Sn_f, Sn_w);

% PSD for continuous model
S= blkdiag(Sv, Sn);

% ---------------- Read data ----------------
file= strcat('../DATA_MOVE/1line_hand_2000Hz/20180403_1.txt');

[~, gyrox, gyroy, gyroz, accx, accy, accz,...
    ~, ~, ~, gyroSts, accSts, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_IMU= length(accx);
N_GNSS= round( N_IMU*dT_IMU / dT_GNSS );

% Rotate to body frame (NED)
R_NB= R_NB_rot(0,deg2rad(180),0);
R_NB_init= blkdiag(R_NB,R_NB);
u= R_NB_init * u;
accx= u(1,:);  accy= u(2,:);  accz= u(3,:);
gyrox= u(4,:); gyroy= u(5,:); gyroz= u(6,:);

% G estimation
g_val= 9.7226; %9.8593 %9.80279
g_N= [0; 0; g_val];

% Initial biases (gravity excluded)
b0= [-0.0193; -0.0005; 0.0000; 0.0003; 0.0001; 0.0000];
% b0= [-0.0319; 0.2930; 0.0045; 0.0009; 0.0002; -0.0007];
% b0= [ 0.0880; -0.1269; 0        ; -0.0011; 0.0002; 0.0006]; % mine

% Allocate variables
P_store= zeros(15, N_GNSS);
P_store_time= zeros(1,N_GNSS);
x= zeros(15,N_IMU);

% Initialize estimate
P= zeros(15);
x(:,1)= zeros(15,1);
x(10:15,1)= b0;        % Initial biases

% Initialize loop variables
timeSim= 0;
timeSum= 0;
k_update= 1;

% Compute the F and G matrices (linear continuous time)
[F,G]= FG_fn(u(1,1),u(2,1),u(3,1),u(5,1),u(6,1),...
             x(7,1),x(8,1),x(9,1),x(10,1),x(11,1),x(12,1),x(14,1),x(15,1),tau,tau);

% Discretize system for IMU (only for variance calculations)
[Phi,D_bar]= discretize(F, G, H, S, dT_IMU);

% --------------------- LOOP ---------------------
for k= 1:N_IMU-1
    
    % Increase time count
    timeSim= timeSim + dT_IMU;
%     timeSum= timeSum + dT_IMU;    
    
    % Update position mean
    x(:,k+1)= IMU_update(x(:,k),u(:,k),g_N,tau,dT_IMU);
%     % Update cov matrix
%     P= Phi*P*Phi' + D_bar;
%     
%     % KF update
%     if timeSum >= dT_GNSS
%         
%         % Compute the F and G matrices (linear continuous time)
%         [F,G]= FG_fn(u(1,k),u(2,k),u(3,k),u(5,k),u(6,k),x(7,k+1),x(8,k+1),x(9,k+1),x(10,k+1),x(11,k+1),x(12,k+1),x(14,k+1),x(15,k+1),tau,tau);
%         
%         % Discretize system for IMU time (only for variance calculations)
%         [Phi,D_bar]= discretize(F, G, H, S, dT_IMU);
%                 
%         % Store cov matrix
%         P_store(:,k_update) = diag(P);
%         P_store_time(k_update)= timeSim;
%         
%         % Time counters
%         timeSum= 0;
%         k_update= k_update+1;
%     end
%     
end
% Store final variance
P_store(:, k_update)= diag(P);
P_store_time(k_update)= timeSim;
% --------------------- END LOOP ---------------------

x_time= 0:dT_IMU:timeSim+dT_IMU/2;


% Plot estimates
figure; hold on; grid on;
plot3(x(1,:),x(2,:),x(3,:),'b.');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis equal

% Plot estimates
figure; hold on;

subplot(3,3,1); hold on;
plot(x_time,x(1,:));
ylabel('x [m]');

subplot(3,3,2); hold on;
plot(x_time,x(2,:));
ylabel('y [m]');

subplot(3,3,3); hold on;
plot(x_time,x(3,:))
ylabel('z [m]');

subplot(3,3,4); hold on;
plot(x_time,x(4,:))
ylabel('v_x [m/s]');

subplot(3,3,5); hold on;
plot(x_time,x(5,:))
ylabel('v_y [m/s]');

subplot(3,3,6); hold on;
plot(x_time,x(6,:));
ylabel('v_z [m/s]');

subplot(3,3,7); hold on;
plot(x_time,rad2deg(x(7,:)))
ylabel('\phi [deg]');

subplot(3,3,8); hold on;
plot(x_time,rad2deg(x(8,:)))
ylabel('\theta [deg]');

subplot(3,3,9); hold on;
plot(x_time,rad2deg(x(9,:)))
ylabel('\psi [deg]');


% check plot
figure; hold on;
plot(x_time, accx)
plot(x_time, accy)
plot(x_time, accz)


%{
SD= 3*sqrt(P_store(:,1:k_update));
P_store_time= P_store_time(1:k_update);

% Plots
subplot(3,3,1); hold on;
plot(P_store_time, SD(1,:), 'r.')
plot(P_store_time, -SD(1,:), 'r.')

subplot(3,3,2); hold on;
plot(P_store_time, SD(2,:), 'r.')
plot(P_store_time, -SD(2,:), 'r.')

subplot(3,3,3); hold on;
plot(P_store_time, SD(3,:), 'r.')
plot(P_store_time, -SD(3,:), 'r.')

subplot(3,3,4); hold on;
plot(P_store_time, SD(4,:), 'r.')
plot(P_store_time, -SD(4,:), 'r.')

subplot(3,3,5); hold on;
plot(P_store_time, SD(5,:), 'r.')
plot(P_store_time, -SD(5,:), 'r.')

subplot(3,3,6); hold on;
plot(P_store_time, SD(6,:), 'r.')
plot(P_store_time, -SD(6,:), 'r.')

subplot(3,3,7); hold on;
plot(P_store_time,rad2deg(x(7,1) + SD(7,:)), 'r.')
plot(P_store_time,rad2deg(x(7,1) - SD(7,:)), 'r.')

subplot(3,3,8); hold on;
plot(P_store_time,rad2deg(x(8,1) + SD(8,:)), 'r.')
plot(P_store_time,rad2deg(x(8,1) - SD(8,:)), 'r.')

subplot(3,3,9); hold on;
plot(P_store_time,rad2deg(SD(9,:)), 'r.')
plot(P_store_time,-rad2deg(SD(9,:)), 'r.')
%}



