
classdef DataClass < handle
    properties
        pred
        update
        im
        
        msmts
        landmarks
        
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= DataClass(imu_num_readings, gps_num_readings)
            obj.pred= PredictionDataClass(imu_num_readings);
            obj.update= UpdateDataClass(imu_num_readings);
            obj.im= IntegrityDataClass(gps_num_readings * 10);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_prediction(obj, epoch, estimator, time)
            obj.pred.store(epoch, estimator, time);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function epoch= store_update(obj, epoch, estimator, time)
            obj.update.store(epoch, estimator, time);
            obj.landmarks{epoch}= estimator.XX(16:end); % store the current landmarks
            
            % increase counter
            epoch= epoch + 1;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_msmts(obj, msmts) % TODO: optimize this mess!!
            obj.msmts= [obj.msmts; msmts];
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_integrity_data(obj, im, counters, params)
            obj.im.store(im, counters, params);
        end
        % ----------------------------------------------
        % ----------------------------------------------
            
        % ----------------------------------------------
        % ----------------------------------------------
        function delete_extra_allocated_memory(obj, counters)
            obj.update.delete_extra_allocated_memory(counters);
            obj.im.delete_extra_allocated_memory(counters);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_map_slam(obj, gps, num_readings, params)
            % Plot GPS+IMU estimated path

            figPath= figure; hold on; grid on;
            plot3(obj.pred.XX(1,:), obj.pred.XX(2,:), obj.pred.XX(3,:), 'b.');
            plot3(obj.update.XX(1,:), obj.update.XX(2,:), obj.update.XX(3,:),...
                'b.','markersize', 7);
            plot3(gps.msmt(1,:),gps.msmt(2,:),gps.msmt(3,:),'r*');
            if params.SWITCH_LIDAR_UPDATE % Plot landmarks
                % create a map of landmarks
                lm_map= [obj.landmarks{end}(1:2:end),...
                         obj.landmarks{end}(2:2:end),...
                         zeros( length(obj.landmarks{end})/2, 1 )];
                plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'g+', 'markersize',20);
                plot3(obj.msmts(:,1), obj.msmts(:,2), zeros(size(obj.msmts,1),1), 'k.');
            end
            
            % plot attitude every 100 IMU readings
            for i= 1:num_readings
                if rem(i,100) == 0
                    R_NB= R_NB_rot(obj.pred.XX(7,i), obj.pred.XX(8,i), obj.pred.XX(9,i));
                    xyz_N= R_NB*params.xyz_B + obj.pred.XX(1:3,i);
                    plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
                end
            end
            xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
            axis equal
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_map_localization(obj, estimator, gps, num_readings, params)
            % Plot GPS+IMU estimated path

            figPath= figure; hold on; grid on;
            plot3(obj.pred.XX(1,:), obj.pred.XX(2,:), obj.pred.XX(3,:), 'b.');
            plot3(obj.update.XX(1,:), obj.update.XX(2,:), obj.update.XX(3,:),...
                'b.','markersize', 7);
            plot3(gps.msmt(1,:),gps.msmt(2,:),gps.msmt(3,:),'r*');
            if params.SWITCH_LIDAR_UPDATE % Plot landmarks
                % create a map of landmarks
                lm_map= [estimator.landmark_map(:,1),...
                         estimator.landmark_map(:,2),...
                         zeros(estimator.num_landmarks,1)];
                plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'g+', 'markersize',20);
                plot3(obj.msmts(:,1), obj.msmts(:,2), zeros(size(obj.msmts,1),1), 'k.');
            end
            
            % plot attitude every 100 IMU readings
            for i= 1:num_readings
                if rem(i,100) == 0
                    R_NB= R_NB_rot(obj.pred.XX(7,i), obj.pred.XX(8,i), obj.pred.XX(9,i));
                    xyz_N= R_NB*params.xyz_B + obj.pred.XX(1:3,i);
                    plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
                end
            end
            xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
            axis equal
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_estimates(obj)
            
            % Plot variance estimates
            standard_dev= sqrt( obj.update.PX );
            
            % Plot SD -- pose
            figure; hold on; title('Standard Deviations');
            
            subplot(3,3,1); hold on; grid on;
            plot(obj.update.time, standard_dev(1,:),'b-','linewidth',2);
            ylabel('x [m]');
            
            subplot(3,3,2); hold on; grid on;
            plot(obj.update.time, standard_dev(2,:),'r-','linewidth',2);
            ylabel('y [m]');
            
            subplot(3,3,3); hold on; grid on;
            plot(obj.update.time, standard_dev(3,:),'g-','linewidth',2);
            ylabel('z [m]');
            
            subplot(3,3,4); hold on; grid on;
            plot(obj.update.time, standard_dev(4,:),'b-','linewidth',2);
            ylabel('v_x [m/s]');
            
            subplot(3,3,5); hold on; grid on;
            plot(obj.update.time, standard_dev(5,:),'r-','linewidth',2);
            ylabel('v_y [m/s]');
            
            subplot(3,3,6); hold on; grid on;
            plot(obj.update.time, standard_dev(6,:),'g-','linewidth',2);
            ylabel('v_z [m/s]');
            
            subplot(3,3,7); hold on; grid on;
            plot(obj.update.time, rad2deg(standard_dev(7,:)),'b-','linewidth',2);
            ylabel('\phi [deg]'); xlabel('Time [s]');
            
            subplot(3,3,8); hold on; grid on;
            plot(obj.update.time, rad2deg(standard_dev(8,:)),'r-','linewidth',2);
            ylabel('\theta [deg]'); xlabel('Time [s]');
            
            subplot(3,3,9); hold on; grid on;
            plot(obj.update.time, rad2deg(standard_dev(9,:)),'g-','linewidth',2);
            ylabel('\psi [deg]'); xlabel('Time [s]');
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_integrity_risk(obj)
            figure; hold on; grid on;
            plot(obj.im.time, obj.im.p_hmi, 'b-', 'linewidth', 2)
%             plot(obj.im.time, obj.im.p_eps, 'r-', 'linewidth', 2)
            set(gca, 'YScale', 'log')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_of_landmarks_in_preceding_horizon(obj)
            figure; hold on; grid on;
            plot(obj.im.time, obj.im.n_L_M, 'b-', 'linewidth', 2)
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end





% timeComplete= 0:params.dT_IMU:timeSim+params.dT_IMU/2;
% timeMove= timeComplete(params.numEpochStatic:end);

% % Plot estimates
% figure; hold on;
% 
% subplot(3,3,1); hold on; grid on;
% plot(timeComplete,DATA.pred.XX(1,:));
% ylabel('x [m]');
% 
% subplot(3,3,2); hold on; grid on;
% plot(timeComplete,DATA.pred.XX(2,:));
% ylabel('y [m]');
% 
% subplot(3,3,3); hold on; grid on; 
% plot(timeComplete,DATA.pred.XX(3,:))
% ylabel('z [m]');
% 
% subplot(3,3,4); hold on; grid on;
% plot(timeComplete,DATA.pred.XX(4,:))
% ylabel('v_x [m/s]');
% 
% subplot(3,3,5); hold on; grid on;
% plot(timeComplete,DATA.pred.XX(5,:))
% ylabel('v_y [m/s]');
% 
% subplot(3,3,6); hold on; grid on;
% plot(timeComplete,DATA.pred.XX(6,:));
% ylabel('v_z [m/s]');
% 
% subplot(3,3,7); hold on; grid on;
% plot(timeComplete,rad2deg(DATA.pred.XX(7,:)))
% ylabel('\phi [deg]');
% 
% subplot(3,3,8); hold on; grid on;
% plot(timeComplete,rad2deg(DATA.pred.XX(8,:)))
% ylabel('\theta [deg]');
% 
% subplot(3,3,9); hold on; grid on;
% plot(timeComplete,rad2deg(DATA.pred.XX(9,:)))
% ylabel('\psi [deg]');


% % Plot variance estimates
% SD= sqrt( data_obj.update.PX(:,1:k_update) );
% update_time= data_obj.update.time(1:k_update);
% 
% % Plot SD -- pose
% figure; hold on; title('Standard Deviations');
% 
% subplot(3,3,1); hold on; grid on;
% plot(update_time, SD(1,:),'b-','linewidth',2);
% ylabel('x [m]');
% 
% subplot(3,3,2); hold on; grid on;
% plot(update_time, SD(2,:),'r-','linewidth',2);
% ylabel('y [m]');
% 
% subplot(3,3,3); hold on; grid on;
% plot(update_time, SD(3,:),'g-','linewidth',2);
% ylabel('z [m]');
% 
% subplot(3,3,4); hold on; grid on;
% plot(update_time, SD(4,:),'b-','linewidth',2);
% ylabel('v_x [m/s]');
% 
% subplot(3,3,5); hold on; grid on;
% plot(update_time, SD(5,:),'r-','linewidth',2);
% ylabel('v_y [m/s]');
% 
% subplot(3,3,6); hold on; grid on;
% plot(update_time, SD(6,:),'g-','linewidth',2);
% ylabel('v_z [m/s]');
% 
% subplot(3,3,7); hold on; grid on;
% plot(update_time, rad2deg(SD(7,:)),'b-','linewidth',2);
% ylabel('\phi [deg]'); xlabel('Time [s]');
% 
% subplot(3,3,8); hold on; grid on;
% plot(update_time, rad2deg(SD(8,:)),'r-','linewidth',2);
% ylabel('\theta [deg]'); xlabel('Time [s]');
% 
% subplot(3,3,9); hold on; grid on;
% plot(update_time, rad2deg(SD(9,:)),'g-','linewidth',2);
% ylabel('\psi [deg]'); xlabel('Time [s]');






% % Plot SD -- Biases
% figure; hold on;
% 
% subplot(2,3,1); hold on; grid on;
% plot(update_time, SD(10,:),'b-','linewidth',2);
% ylabel('a_x');
% 
% subplot(2,3,2); hold on; grid on;
% plot(update_time, SD(11,:),'r-','linewidth',2);
% ylabel('a_y');
% 
% subplot(2,3,3); hold on; grid on;
% plot(update_time, SD(12,:),'g-','linewidth',2);
% ylabel('a_z');
% 
% subplot(2,3,4); hold on; grid on;
% plot(update_time, SD(13,:),'b--','linewidth',2);
% ylabel('w_x'); xlabel('Time [s]')
% 
% subplot(2,3,5); hold on; grid on;
% plot(update_time, SD(14,:),'r--','linewidth',2);
% ylabel('w_y'); xlabel('Time [s]')
% 
% subplot(2,3,6); hold on; grid on;
% plot(update_time, SD(15,:),'g--','linewidth',2);
% ylabel('w_z'); xlabel('Time [s]')


% % Plot biases
% figure; hold on; grid on; title('biases in accelerometers');
% plot(timeComplete, DATA.pred.XX(10,:), 'linewidth',2)
% plot(timeComplete, DATA.pred.XX(11,:), 'linewidth',2)
% plot(timeComplete, DATA.pred.XX(12,:), 'linewidth',2)
% ylabel('m/s^2')
% legend('x','y','z')
% 
% figure; hold on; grid on; title('biases in gyros');
% plot(timeComplete, rad2deg(DATA.pred.XX(13,:)), 'linewidth',2)
% plot(timeComplete, rad2deg(DATA.pred.XX(14,:)), 'linewidth',2)
% plot(timeComplete, rad2deg(DATA.pred.XX(15,:)), 'linewidth',2)
% ylabel('deg');
% legend('w_x','w_y','w_z')


% % Plot GPS positions
% figure; hold on; grid on; title('GPS positions');
% plot(1:gps.num_readings, gps.msmt(1,:), 'linewidth',2)
% plot(1:gps.num_readings, gps.msmt(2,:), 'linewidth',2)
% plot(1:gps.num_readings, gps.msmt(3,:), 'linewidth',2)
% ylabel('m')
% legend('r_x','r_y','r_z')

% % Plot GPS velocities
% figure; hold on; grid on; title('GPS velocities');
% plot(1:gps.num_readings, gps.msmt(4,:), 'linewidth',2)
% plot(1:gps.num_readings, gps.msmt(5,:), 'linewidth',2)
% plot(1:gps.num_readings, gps.msmt(6,:), 'linewidth',2)
% ylabel('m/s')
% legend('v_x','v_y','v_z')


% % Plot measurements
% figure; hold on; grid on;
% u_ax_filter= filter(ones(1,500)/500,1,u(1,:));
% u_ay_filter= filter(ones(1,500)/500,1,u(2,:));
% plot(timeComplete, u_ax_filter(:));
% plot(timeComplete, u_ay_filter(:));
% legend('accX','accY')




