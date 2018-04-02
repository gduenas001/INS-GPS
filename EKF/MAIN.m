clear; format short; clc; close all;

% Parameters
NepochsInit= 5000; % Number of epochs used to calcualte bias
R= 0.001^2 * eye(3); % Measurement variance matrix
dT_GNSS= 1; % KF Update period
dt_IMU= 1/2000;
SWITCH_KF_UPDATE= 0;
SWITCH_INTEGRATION= 1; % If 1, integrate in continuous time,
                       % If 2, direct integration without linearization

% Convert from sensor-frame to body-frame (NED)
R_SENSOR2BODY = zeros(6);
R_SENSOR2BODY(4:6,4:6)= [-1 0 0;
                          0 1 0;
                          0 0 -1];
R_SENSOR2BODY(1:3,1:3)= [-1 0 0;
                          0 1 0;
                          0 0 -1];

% Tau for bias -- from manufacturer
tau= 3600;
BiasInstabW= deg2rad(0.3/3600);     % rad/s
BiasInstabA= 0.05 * 9.80665 / 1000; % m/s2

% Measured white noise SD
sigma_IMU= [0.019342455000080;  % ax
            0.018660194431183;  % ay
            0.019979928602605;  % az
            0.001239820655196;  % wx [rad]
            0.001353946031205;  % wy [rad]
            0.001297525572326]; % wx [rad]
       
% Convert to PSD
sigma_IMU= sigma_IMU * sqrt(dt_IMU);

% PSD for continuous model
Qw= diag([sigma_IMU', ...
    BiasInstabA, BiasInstabA, BiasInstabA, ...
    BiasInstabW, BiasInstabW, BiasInstabW]).^2;


testInit= 1; testLast = 5;
for testNum= testInit:testLast
%     disp(testNum);
    
    % ---------------- Read data ---------------- 
    file= strcat('../DATA_STATIC/24x120k/20180228_12', num2str(testNum),'.txt');
%     file= strcat('../DATA_STATIC/1x20k_125Hz/20180326_184641_1.txt');
    [timeIMU, gyrox, gyroy, gyroz, accx, accy, accz,...
        ~, ~, ~, gyroSts, accSts, ~, ~, ~, ~]= DataRead(file); % rads
    % -------------------------------------------

    % Set paramters
    u= [accx, accy, accz, gyrox, gyroy, gyroz]';
    dt= diff(timeIMU);
    timeIMU_final= timeIMU(end);
    N_IMU= length(timeIMU);
    N_GNSS= floor(timeIMU(end)/dT_GNSS);
    
    % Rotate to body frame (NED)
    u= R_SENSOR2BODY * u;
    accx= u(1,:);  accy= u(2,:);  accz= u(3,:);
    gyrox= u(4,:); gyroy= u(5,:); gyroz= u(6,:);
    
    % Initial values/biases
    ax0= mean(accx(1:NepochsInit));
    ay0= mean(accy(1:NepochsInit));
    az0= mean(accz(1:NepochsInit));
    phi0= atan2(-ay0,-az0);
    theta0= atan2(ax0, sqrt(ay0^2 + az0^2));
    
    % G estimation
    g_val= sqrt(ax0^2 + ay0^2 + az0^2);
    G_N= [0; 0; g_val];
    
    % Gyro bias estimation
    wx0= mean(gyrox(1:NepochsInit));
    wy0= mean(gyroy(1:NepochsInit));
    wz0= mean(gyroz(1:NepochsInit));  
    
    % Initial biases
    u0= [ax0; ay0; az0; wx0; wy0; wz0];
    
    % Allocate variables
    Pk_store= zeros(15, N_GNSS);
    Pk= zeros(15);
    x= zeros(15,N_IMU);
    x_dot_star= zeros(15,1);
    u_star= zeros(6,1); u_star(3)= -g_val; % expected IMU reading
%     u_star= u0;
    x(:,1)= zeros(15,1); 
    x(7,1)= phi0; x(8,1)= theta0; % Initial pose
    x(10:15,1)= u0; x(12,1)= x(12,1)+g_val; % Initial biases
    x_star= x(:,1);
    t_sum= 0;
    k_update= 1;
    
    % Initial linearization & discretization
    [~, F, Gu_tilda, Gw, H]=...
        Matrices(x_star, u_star(1), u_star(2), u_star(3), u_star(4), u_star(5), u_star(6), tau);
    [Phi, Gamma, Gammaw_W_Gammaw]= Matrices2Discrete( F, Gu_tilda, Gw, H, Qw, dt_IMU );
    
    % --------------------- LOOP ---------------------
    for k= 1:N_IMU-1
        if SWITCH_INTEGRATION == 1 % Simple integration
            Deltax_dot= F*( x(:,k) - x_star ) + Gu_tilda*( u(:,k) - u_star );
            x_dot= x_dot_star + Deltax_dot;
            x(:,k+1)= x(:,k) + dt(k)*x_dot;
        elseif SWITCH_INTEGRATION == 2 % Direct integration without linearization
            x(:,k+1)= IMU_update(x(:,k),u(:,k),G_N,tau,dt(k));
        else
            % Discretize version integration (assumes IMU sampling freq is constant)
            Deltax= Phi*( x(:,k) - x_star ) + Gamma*( u(:,k) - u_star );
            x(:,k+1)= x_star + Deltax;
        end
        
        % KF update
        if t_sum >= dT_GNSS
            
            % Store variances
            Pk_store(:,k_update) = diag(Pk);
            
            % Linearization points
            x_star= x(:,k+1);
            u_star= u(:,k);
            
            % Linearize & discretize system for GNSS upate time (only for variance calculations)
            [~, F, Gu_tilda, Gw, H]=...
                Matrices(x_star, u_star(1), u_star(2), u_star(3), u_star(4), u_star(5), u_star(6), tau);
            [Phi, Gamma, Gammaw_W_Gammaw]= Matrices2Discrete( F, Gu_tilda, Gw, H, Qw, dT_GNSS );
            
            % Update variance
            Pk= Phi*Pk*Phi' + Gammaw_W_Gammaw;
            
            % GNSS update
            if SWITCH_KF_UPDATE
                L= Pk*H'/(H*Pk*H' + R);
                y= [0; 0; 0];
                y_hat= H*x(:,k+1);
                innov= y - y_hat;
                x(:,k+1)= x(:,k+1) + L*innov;
                Pk= Pk - L*H*Pk;
            end
            t_sum= 0;
            k_update= k_update+1;
            
            % New linearization point after GNSS update
            x_star= x(:,k+1);
            
            % Linearize & discretize system for IMU freq
            [~, F, Gu_tilda, Gw, H]=...
                Matrices(x_star, u_star(1), u_star(2), u_star(3), u_star(4), u_star(5), u_star(6), tau);

            if SWITCH_INTEGRATION == 1
                x_dot_star= x_dot; % linearization point for x_dot
            else % Discretize for IMU freq
                [Phi, Gamma, Gammaw_W_Gammaw]= Matrices2Discrete( F, Gu_tilda, Gw, H, Qw, dt_IMU);
            end
        end
        t_sum= t_sum + dt(k);
                
    end
    % Store final variance
    Pk_store(:, k_update)= diag(Pk);
    
    % Plot errors for this run
    figure(SWITCH_KF_UPDATE+1); hold on;
    
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

TT = dT_GNSS:dT_GNSS:timeIMU_final;
std = 3*sqrt(Pk_store);

% Plots
subplot(3,3,1); hold on;
plot(TT, std(1,:), 'r.')
plot(TT, -std(1,:), 'r.')

subplot(3,3,2); hold on;
plot(TT, std(2,:), 'r.')
plot(TT, -std(2,:), 'r.')

subplot(3,3,3); hold on;
plot(TT, std(3,:), 'r.')
plot(TT, -std(3,:), 'r.')

subplot(3,3,4); hold on;
plot(TT, std(4,:), 'r.')
plot(TT, -std(4,:), 'r.')

subplot(3,3,5); hold on;
plot(TT, std(5,:), 'r.')
plot(TT, -std(5,:), 'r.')

subplot(3,3,6); hold on;
plot(TT, std(6,:), 'r.')
plot(TT, -std(6,:), 'r.')

subplot(3,3,7); hold on;
plot(TT,rad2deg(x(7,1) + std(7,:)), 'r.')
plot(TT,rad2deg(x(7,1) - std(7,:)), 'r.')

subplot(3,3,8); hold on;
plot(TT,rad2deg(x(8,1) + std(8,:)), 'r.')
plot(TT,rad2deg(x(8,1) - std(8,:)), 'r.')

subplot(3,3,9); hold on;
plot(TT,rad2deg(std(9,:)), 'r.')
plot(TT,-rad2deg(std(9,:)), 'r.')




