clear; format short; clc; close all;

% Parameters
NepochsInit= 5000; % Number of epochs used to calcualte bias
R= 0.001^2 * eye(3); % Measurement variance matrix
dT_GNSS= 1; % KF Update period
dT_IMU= 1/125 ;
SWITCH_KF_UPDATE= 0;

% Observation matrix
H= [eye(3), zeros(3,12)];

% % Measured white noise SD
% sigma_IMU= [0.019342455000080;  % ax
%             0.018660194431183;  % ay
%             0.019979928602605;  % az
%             0.001239820655196;  % wx [rad]
%             0.001353946031205;  % wy [rad]
%             0.001297525572326]; % wx [rad]
% V= diag(sigma_IMU).^2;
      
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
sn_f= ( 0.05 * 9.80665 / 1000 )^2; Sn_f= diag([sn_f, sn_f, sn_f]);
sn_w= ( deg2rad(0.3/3600) )^2;     Sn_w= diag([sn_w, sn_w, sn_w]);
Sn= blkdiag(Sn_f, Sn_w);

% PSD for continuous model
S= blkdiag(Sv, Sn); 

% Convert from sensor-frame to body-frame (NED)
R_NB= R_NB_rot(0,deg2rad(180),0);
R_NB_init= blkdiag(R_NB,R_NB);

testInit= 1; testLast= 5;
for testNum= testInit:testLast
    % ---------------- Read data ---------------- 
%     file= strcat('../DATA_STATIC/24x120k/20180228_12', num2str(testNum),'.txt');
%     file= strcat('../DATA_STATIC/1x20k_125Hz/20180326_184641_1.txt');
%     file= strcat('../DATA_STATIC/5x1min_125Hz/20180327_', num2str(testNum),'.txt');
    file= strcat('../DATA_STATIC/5x1min_250Hz/20180327_', num2str(testNum),'.txt');

    [timeIMU, gyrox, gyroy, gyroz, accx, accy, accz,...
        ~, ~, ~, gyroSts, accSts, ~, ~, ~, ~]= DataRead(file); % rads
    % -------------------------------------------

    % Set paramters
    u= [accx, accy, accz, gyrox, gyroy, gyroz]';
    dt= diff(timeIMU);
    N_IMU= length(timeIMU);
    N_GNSS= floor(timeIMU(end)/dT_GNSS);
    
    % Rotate to body frame (NED)
    u= R_NB_init * u;
    accx= u(1,:);  accy= u(2,:);  accz= u(3,:);
    gyrox= u(4,:); gyroy= u(5,:); gyroz= u(6,:);
    
    % Initial values/biases
    ax0= mean(accx(1:NepochsInit));
    ay0= mean(accy(1:NepochsInit));
    az0= mean(accz(1:NepochsInit));
    phi0= atan2(-ay0,-az0);
    theta0= atan2(ax0, sqrt(ay0^2 + az0^2));
    
    % G estimation
%     g_val= 9.80279; 
    g_val= sqrt(ax0^2 + ay0^2 + az0^2);
    g_N= [0; 0; g_val]; 
    
    % Gyro bias estimation
    wx0= mean(gyrox(1:NepochsInit));
    wy0= mean(gyroy(1:NepochsInit));
    wz0= mean(gyroz(1:NepochsInit));  
    
    % Initial biases
    b0= [ax0; ay0; az0 + g_val; wx0; wy0; wz0];
    
    % Allocate variables
    P_store= zeros(15, N_GNSS);
    P_store_time= zeros(1,N_GNSS);
    x= zeros(15,N_IMU);
    
    % Initialize estimate
    P= zeros(15);
    x(:,1)= zeros(15,1); 
    x(10:15,1)= b0;        % Initial biases
    
    % Initialize loop variables
    timeSim= timeIMU(1);
    timeSum= 0;
    k_update= 1;
    
    % Compute the F and G matrices (linear continuous time)
    [F,G]= FG_fn(u(1,1),u(2,1),u(3,1),u(5,1),u(6,1),x(7,1),x(8,1),x(9,1),x(10,1),x(11,1),x(12,1),x(14,1),x(15,1),tau,tau);
    
    % Discretize system for IMU (only for variance calculations)
    [Phi,D_bar]= discretize(F, G, H, S, dT_IMU);
     
    % --------------------- LOOP ---------------------
    for k= 1:N_IMU-1
        % Update position mean
        x(:,k+1)= IMU_update(x(:,k),u(:,k),g_N,tau,dt(k));
        % Update cov matrix
        P= Phi*P*Phi' + D_bar;
        
        % KF update
        if timeSum >= dT_GNSS
            
            % Store cov matrix
            P_store(:,k_update) = diag(P);
            P_store_time(k_update)= timeSim;

            % Compute the F and G matrices (linear continuous time)
            [F,G]= FG_fn(u(1,k),u(2,k),u(3,k),u(5,k),u(6,k),x(7,k+1),x(8,k+1),x(9,k+1),x(10,k+1),x(11,k+1),x(12,k+1),x(14,k+1),x(15,k+1),tau,tau);
            
            % Discretize system for GNSS upate time (only for variance calculations)
            [Phi,D_bar]= discretize(F, G, H, S, dT_IMU);
                        
            % GNSS update
            if SWITCH_KF_UPDATE
                L= P*H'/(H*P*H' + R);
                y= [0; 0; 0];
                y_hat= H*x(:,k+1);
                innov= y - y_hat;
                x(:,k+1)= x(:,k+1) + L*innov;
                P= P - L*H*P;
            end
            % Time counters
            timeSum= 0;
            k_update= k_update+1;
        end
        
        % Increase time count
        timeSim= timeSim + dt(k);
        timeSum= timeSum + dt(k);
                
    end
    % --------------------- END LOOP ---------------------
    
    % Store final variance
    P_store(:, k_update)= diag(P);
    P_store_time(k_update)= timeSim;
    
    % Plot errors for this run
    figure(1); hold on;
    
    subplot(3,3,1); hold on;
    plot(timeIMU,x(1,:));
    ylabel('x [m]');
    
    subplot(3,3,2); hold on;
    plot(timeIMU,x(2,:));
    ylabel('y [m]');
    
    subplot(3,3,3); hold on;
    plot(timeIMU,x(3,:))
    ylabel('z [m]');
    
    subplot(3,3,4); hold on;
    plot(timeIMU,x(4,:))
    ylabel('v_x [m/s]');
    
    subplot(3,3,5); hold on;
    plot(timeIMU,x(5,:))
    ylabel('v_y [m/s]');
    
    subplot(3,3,6); hold on;
    plot(timeIMU,x(6,:));
    ylabel('v_z [m/s]');
    
    subplot(3,3,7); hold on;
    plot(timeIMU,rad2deg(x(7,:)))
    ylabel('\phi [deg]');
    
    subplot(3,3,8); hold on;
    plot(timeIMU,rad2deg(x(8,:)))
    ylabel('\theta [deg]');
    
    subplot(3,3,9); hold on;
    plot(timeIMU,rad2deg(x(9,:)))
    ylabel('\psi [deg]');
    
end

% P_store_time= dT_GNSS:dT_GNSS:timeIMU(end);
std= 3*sqrt(P_store(:,1:k_update));
P_store_time= P_store_time(1:k_update);

% Plots
subplot(3,3,1); hold on;
plot(P_store_time, std(1,:), 'r.')
plot(P_store_time, -std(1,:), 'r.')

subplot(3,3,2); hold on;
plot(P_store_time, std(2,:), 'r.')
plot(P_store_time, -std(2,:), 'r.')

subplot(3,3,3); hold on;
plot(P_store_time, std(3,:), 'r.')
plot(P_store_time, -std(3,:), 'r.')

subplot(3,3,4); hold on;
plot(P_store_time, std(4,:), 'r.')
plot(P_store_time, -std(4,:), 'r.')

subplot(3,3,5); hold on;
plot(P_store_time, std(5,:), 'r.')
plot(P_store_time, -std(5,:), 'r.')

subplot(3,3,6); hold on;
plot(P_store_time, std(6,:), 'r.')
plot(P_store_time, -std(6,:), 'r.')

subplot(3,3,7); hold on;
plot(P_store_time,rad2deg(x(7,1) + std(7,:)), 'r.')
plot(P_store_time,rad2deg(x(7,1) - std(7,:)), 'r.')

subplot(3,3,8); hold on;
plot(P_store_time,rad2deg(x(8,1) + std(8,:)), 'r.')
plot(P_store_time,rad2deg(x(8,1) - std(8,:)), 'r.')

subplot(3,3,9); hold on;
plot(P_store_time,rad2deg(std(9,:)), 'r.')
plot(P_store_time,-rad2deg(std(9,:)), 'r.')




