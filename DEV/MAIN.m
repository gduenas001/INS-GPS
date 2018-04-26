
clear; format short; clc; close all;

configureFile;

% Initial discretization for cov. propagation
[Phi,D_bar]= linearize_discretize(x(:,1),u(:,1),S,taua,tauw,dT_IMU);

% ----------------------------------------------------------
% -------------------------- LOOP --------------------------
for k= 1:N_IMU-1
    
    % Turn off GPS updates if start moving
    if k == numEpochStatic
        SWITCH_CALIBRATION= 0; 
        P(7,7)= sig_phi0^2;
        P(8,8)= sig_phi0^2;
        P(9,9)= sig_yaw0^2;
        taua= taua0;
        tauw= tauw0;
    end
        
    % Increase time count
    timeSim= timeSim + dT_IMU;
    timeSum= timeSum + dT_IMU;
    timeSumVirt_Z= timeSumVirt_Z + dT_IMU;
    timeSumVirt_Y= timeSumVirt_Y + dT_IMU;
    
    % ------------- IMU -------------
    x(:,k+1)= IMU_update( x(:,k), u(:,k), g_N, taua, tauw, dT_IMU );
    P= Phi*P*Phi' + D_bar; 
    % -------------------------------
    
    % ------------- Calibration -------------
    if timeSum >= dT_cal && SWITCH_CALIBRATION
        
        z= [zeros(6,1); phi0; theta0; yaw0];
        [x(:,k+1),P]= calibration(x(:,k+1),P,z,H_cal,R_cal);
        
        [Phi,D_bar]= linearize_discretize(x(:,k+1),u(:,k),S,taua,tauw,dT_IMU);
        
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
    % ------------------------------------
    
    % ------------- virtual msmt update >> Z vel  -------------  
    if timeSumVirt_Z >= dT_virt_Z && SWITCH_VIRT_UPDATE_Z && ~SWITCH_CALIBRATION
        [x(:,k+1),P]= zVelocityUpdate(x(:,k+1),P,R_virt_Z);
        
        % Reset counter
        timeSumVirt_Z= 0;
    end
    % ---------------------------------------------------------
    
    % ------------- virtual msmt update >> Y vel  -------------  
    if timeSumVirt_Y >= dT_virt_Y && SWITCH_VIRT_UPDATE_Y && ~SWITCH_CALIBRATION        
         
        % Yaw update
        if SWITCH_YAW_UPDATE && norm(x(4:6,k+1)) > minVelocityYaw
            disp('yaw udpate');
            [x(:,k+1),P]= yawUpdate(x(:,k+1),P,u(4:6,k),H_yaw,R_yaw,r_IMU2rearAxis);
        else
            disp('--------no yaw update------');
        end
        
        % Reset counter
        timeSumVirt_Y= 0;
    end
    % ---------------------------------------------------------
    
    % ------------- GPS -------------
    if (timeSim + dT_IMU) > timeGPS 
        
        if ~SWITCH_CALIBRATION && SWITCH_GPS_UPDATE 
            % GPS update -- only use GPS vel if it's fast
            [x(:,k+1),P]= GPS_update(x(:,k+1),P,z_GPS(:,k_GPS),R_GPS(:,k_GPS),...
                                     minVelocityGPS,SWITCH_GPS_VEL_UPDATE);
            
            % Yaw update 
            if SWITCH_YAW_UPDATE && norm(x(4:6,k+1)) > minVelocityYaw
                disp('yaw udpate');
                [x(:,k+1),P]= yawUpdate(x(:,k+1),P,u(4:6,k),H_yaw,R_yaw,r_IMU2rearAxis);
            else
                disp('--------no yaw update------');
            end      
        end
        [Phi,D_bar]= linearize_discretize(x(:,k+1),u(:,k),S,taua,tauw,dT_IMU);
                                  
        % Store cov matrix
        P_store(:,k_update)= diag(P);
        P_store_time(k_update)= timeSim;
        k_update= k_update+1;
        
        % Time GPS counter
        k_GPS= k_GPS + 1;
        timeGPS= T_GPS(k_GPS);
    end
    % --------------------------------
    
    % ------------- LIDAR -------------
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
    % ---------------------------------
    
end
% Store final variance
P_store(:, k_update)= diag(P);
P_store_time(k_update)= timeSim;
% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------


% ------------- PLOTS -------------
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
if SWITCH_LIDAR_UPDATE,  plot3(LM(:,1),LM(:,2),LM(:,3),'k.'); end % Plot landmarks
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
plot_attitude( x(1:9,1:100:end), figPath ) % Plot attitude at some epochs
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


% Plot variance estimates
SD= sqrt(P_store(:,1:k_update));
P_store_time= P_store_time(1:k_update);

figure; hold on; grid on;
plot(P_store_time, SD(1,:),'b-','linewidth',2);
plot(P_store_time, SD(2,:),'r-','linewidth',2);
plot(P_store_time, SD(3,:),'g-','linewidth',2);
plot(P_store_time, SD(4,:),'b--','linewidth',2);
plot(P_store_time, SD(5,:),'r--','linewidth',2);
plot(P_store_time, SD(6,:),'g--','linewidth',2);
xlabel('Time epochs')
ylabel('m');
legend('x','y','z','v_x','v_y','v_z');

figure; hold on; grid on;
plot(P_store_time, rad2deg(SD(7,:)),'b-','linewidth',2);
plot(P_store_time, rad2deg(SD(8,:)),'r-','linewidth',2);
plot(P_store_time, rad2deg(SD(9,:)),'g-','linewidth',2);
xlabel('Time epochs')
ylabel('deg');
legend('\phi','\theta','\psi');

% figure; hold on; grid on;
% plot(P_store_time, SD(10,:),'b-','linewidth',2);
% plot(P_store_time, SD(11,:),'r-','linewidth',2);
% plot(P_store_time, SD(12,:),'g-','linewidth',2);
% plot(P_store_time, SD(13,:),'b--','linewidth',2);
% plot(P_store_time, SD(14,:),'r--','linewidth',2);
% plot(P_store_time, SD(15,:),'g--','linewidth',2);
% xlabel('Time epochs')
% ylabel('');
% legend('a_x','a_y','a_z','w_x','w_y','w_z');









