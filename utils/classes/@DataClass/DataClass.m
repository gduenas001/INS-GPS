
classdef DataClass < handle
    properties
        pred
        update
        im
        
        msmts        
        gps_msmts
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= DataClass(imu_num_readings, lidar_num_readings, params)
            obj.pred= PredictionDataClass(imu_num_readings, params);
            obj.update= UpdateDataClass(imu_num_readings, params);
            obj.im= IntegrityDataClass(lidar_num_readings);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_prediction(obj, epoch, estimator, time)
            obj.pred.store(epoch, estimator, time);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_prediction_sim(obj, epoch, estimator, time)
            obj.pred.store_sim(epoch, estimator, time);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function epoch= store_update(obj, epoch, estimator, time)
            obj.update.store(epoch, estimator, time);
            
            % increase counter
            epoch= epoch + 1;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function epoch= store_update_sim(obj, epoch, estimator, time)
            obj.update.store_sim(epoch, estimator, time);
            
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
        function store_gps_msmts(obj, gps_msmt) % TODO: optimize this mess!!
            obj.gps_msmts= [obj.gps_msmts; gps_msmt'];
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_integrity_data(obj, im, estimator, counters, params)
            obj.im.store(im, estimator, counters, params);
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
        lm_map= plot_map_slam(obj, estimator, gps, num_readings, params)
        % ----------------------------------------------
        % ----------------------------------------------
        plot_map_localization(obj, estimator, gps, num_readings, params)
        % ----------------------------------------------
        % ----------------------------------------------
        plot_map_localization_sim(obj, estimator, num_readings, params)
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_MA_probabilities(obj)
            figure; hold on; grid on;
            for i= 1:length(obj.im.time)
                % if it's empty --> continue
                if isempty(obj.im.association), continue, end
                
                % take the landmark indexes
                lm_inds= obj.im.association{i};
                P_MA= obj.im.P_MA_k{i};
                
                % plot
                for j= 1:length(lm_inds)
                    plot( lm_inds(j), P_MA(j), 'bo' )
                end
            end
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_P_H(obj)
            figure; hold on; grid on;
            for i= 1:length(obj.im.time)
                time= obj.im.time(i);
                P_H= obj.im.P_H{i};
                if isempty(P_H), continue, end
                
                for j= 1:length(P_H)
                    plot(time, P_H(j), '.')
                end
            end
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
            ylim([1e-15,1]);
            xlim([obj.im.time(1), obj.im.time(end)]) % reset the x-axis (otherwise it moves)
            xlabel('Time [s]')
            ylabel('P(HMI)')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_of_landmarks_in_preceding_horizon(obj)
            figure; hold on; grid on;
            plot(obj.im.time, obj.im.n_L_M, 'b-', 'linewidth', 2)
            plot(obj.update.time, obj.update.number_of_associated_LMs, 'g-', 'linewidth', 2)
            legend('n_L^M', 'n_L');
            xlabel('time [s]')
            ylabel('Number of landmarks in the preceding horizon')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_epochs_in_preceding_horizon(obj)
            figure; hold on; grid on;
            plot(obj.im.time, obj.im.M, 'b-', 'linewidth', 2)
            xlabel('time [s]')
            ylabel('Number of preceding horizon epochs')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_bias_calibration(obj)
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.XX(10:12,:), 'linewidth',2)
            legend('acc_x','acc_y','acc_z')
            
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.XX(13:15,:), 'linewidth',2)
            legend('w_x','w_y','w_z')
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




