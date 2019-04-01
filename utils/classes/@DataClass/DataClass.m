
classdef DataClass < handle
    properties
        pred
        update
        im
        
        num_extracted_features
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
        function epoch= store_update_fg(obj, epoch, estimator, time, params)
            obj.update.store_fg(epoch, estimator, time, params);
            
            % increase counter
            epoch= epoch + 1;
        end
        function epoch= store_update_sim(obj, epoch, estimator, time, params)
            obj.update.store_sim(epoch, estimator, time, params);
            
            % increase counter
            epoch= epoch + 1;
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function store_msmts(obj, msmts) % TODO: optimize this mess!!
            obj.num_extracted_features= [obj.num_extracted_features; size(msmts,1)];
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
        function delete_extra_allocated_memory(obj, counters)
            obj.update.delete_extra_allocated_memory(counters);
            obj.im.delete_extra_allocated_memory(counters);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function HMI_inds= find_HMI_sim(obj, params)
        % this function finds the indexes where HMI occurs, i.e. where
        % the error surpasses the alert limit and the detector is not
        % triggered
            
            HMI_inds= [];
            fail_ind= find( obj.update.error_state_interest > params.alert_limit );
            
            if isempty(fail_ind), return, end
            
            for i= 1:length(fail_ind)
                if obj.update.q_d(fail_ind(i)) < obj.update.T_d(fail_ind(i))
                    HMI_inds= [HMI_inds; fail_ind(i)];
                end
            end
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
        plot_map_localization_fg(obj, estimator, params)
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_detector(obj, params)
            % plot the detector value Vs. the detector threshold to see if 
            % an alarm has been triggered
            
            figure; hold on; grid on;
            if params.SWITCH_SIM
                plot(obj.im.time * params.velocity_sim, obj.im.detector, 'linewidth', 2)
                plot(obj.im.time * params.velocity_sim, obj.im.detector_threshold, 'linewidth', 2)
                xlabel('x [m]')
            else
                plot(obj.im.time, obj.im.detector, 'linewidth', 2)
                plot(obj.im.time, obj.im.detector_threshold, 'linewidth', 2)
                xlabel('time [s]')
            end
            legend('detector', 'threshold')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_detector_fg(obj, params)
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.q_d, 'linewidth', 2)
            plot(obj.update.time, obj.update.T_d, 'linewidth', 2)
            legend('detector', 'threshold')
        end
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
            xlabel('landmark ID')
            ylabel('P(MA)')
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
            xlabel('time [s]')
            ylabel('P(H)')
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_error(obj, params)
%             standard_dev_x= sqrt( obj.update.PX(1,:) );
            standard_dev_y= sqrt( obj.update.PX(2,:) );
            
            figure; hold on; grid on;
%             plot(obj.update.time * params.velocity_sim, obj.update.error(1,:), 'b-', 'linewidth', 2)
            plot(obj.update.time * params.velocity_sim, obj.update.error(2,:), 'r-', 'linewidth', 2)
%             plot(obj.update.time * params.velocity_sim, standard_dev_x,'b--','linewidth',2);
%             plot(obj.update.time * params.velocity_sim, -standard_dev_x,'b--','linewidth',2);
            plot(obj.update.time * params.velocity_sim, standard_dev_y,'r--','linewidth',2);
            plot(obj.update.time * params.velocity_sim, -standard_dev_y,'r--','linewidth',2);
            
            legend({'$\delta \hat{x}$', '$\hat{\sigma}$'},'interpreter', 'latex','fontsize', 15)
            xlabel('x [m]','interpreter', 'latex','fontsize', 15)
            ylabel('error [m]','interpreter', 'latex','fontsize', 15)
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_error_fg(obj, params)
            
            figure; hold on; grid on;

            plot(obj.update.time,  obj.update.error_state_interest(:), 'r-', 'linewidth', 2)
%             plot(obj.update.time,  obj.update.sig_state_interest(:) ,'r--','linewidth',2);
%             plot(obj.update.time, -obj.update.sig_state_interest(:),'r--','linewidth',2);
            
%             legend({'$\delta \hat{x}$', '$\hat{\sigma}$'},'interpreter', 'latex','fontsize', 15)
            xlabel('time [s]','interpreter', 'latex','fontsize', 15)
            ylabel('error [m]','interpreter', 'latex','fontsize', 15)
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
        function plot_integrity_risk(obj, params)
            figure; hold on; grid on;
            if params.SWITCH_SIM
                plot(obj.im.time * params.velocity_sim, obj.im.p_hmi, 'b-', 'linewidth', 2)
                xlabel('x [m]','interpreter', 'latex','fontsize', 15)
                xlim([obj.im.time(1), obj.im.time(end)] * params.velocity_sim) % reset the x-axis (otherwise it moves)
            else
                if params.SWITCH_FACTOR_GRAPHS
                    plot(obj.im.time, obj.im.p_hmi, 'b-', 'linewidth', 2)
                    xlabel('Time [s]','interpreter', 'latex','fontsize', 15)
                    xlim([obj.im.time(1), obj.im.time(end)]) % reset the x-axis (otherwise it moves)
                else
                    plot(obj.im.time, obj.im.p_hmi, 'b-', 'linewidth', 2)
                    xlabel('Time [s]','interpreter', 'latex','fontsize', 15)
                    xlim([obj.im.time(1), obj.im.time(end)]) % reset the x-axis (otherwise it moves)
                end
            end
            % plot(obj.im.time, obj.im.p_eps, 'r-', 'linewidth', 2)
            ylabel('P(HMI)','interpreter', 'latex','fontsize', 15)
            set(gca, 'YScale', 'log')
            ylim([1e-15,1]);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_of_landmarks(obj, params)
            figure; hold on; grid on;
            if params.SWITCH_SIM
                plot(obj.im.time * params.velocity_sim, obj.im.n_L_M, 'b-', 'linewidth', 2)
                plot(obj.update.time * params.velocity_sim, obj.update.num_associated_lms, 'g-', 'linewidth', 2)
%                 plot(obj.im.time * params.velocity_sim, obj.update.miss_associations, 'r*')
%                 plot(obj.update.time * params.velocity_sim, obj.update.num_of_extracted_features, 'k-', 'linewidth', 2)
                xlabel({'x [m]'},'interpreter', 'latex','fontsize', 15)
                legend({'$n^{F^(M)}$', '$n^F$'},...
                    'interpreter', 'latex','fontsize', 15);
            else
                plot(obj.im.time, obj.im.n_L_M, 'b-', 'linewidth', 2)
                plot(obj.update.time, obj.update.num_associated_lms, 'g-', 'linewidth', 2)
%                 plot(obj.update.time, obj.update.num_of_extracted_features, 'k-', 'linewidth', 2)
                xlabel('time [s]','interpreter', 'latex')
                legend({'$n^{F^(M)}$', '$n^F$'},'interpreter', 'latex','fontsize', 15);
            end
%             ylabel('Number of landmarks','interpreter', 'latex','fontsize', 15)
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_of_landmarks_fg_sim(obj, params)
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.n_L_M, 'b-', 'linewidth', 2)
            plot(obj.update.time, obj.update.n_L_k, 'g-', 'linewidth', 2)
            if ~params.SWITCH_OFFLINE
                plot(obj.update.time, obj.update.num_faults, 'r-', 'linewidth', 2)
            end
            xlabel({'x [m]'},'interpreter', 'latex','fontsize', 15)
            legend({'$n^{L^(M)}$', '$n^L$', '$n_{f}$'},...
                'interpreter', 'latex','fontsize', 15);
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_number_epochs_in_preceding_horizon(obj, params)
            figure; hold on; grid on;
            if params.SWITCH_SIM
                plot(obj.im.time * params.velocity_sim, obj.im.M, 'b-', 'linewidth', 2)
                xlabel('x [m]','interpreter', 'latex','fontsize', 15)
            else
                plot(obj.im.time, obj.im.M, 'b-', 'linewidth', 2)
                xlabel('time [s]','interpreter', 'latex','fontsize', 15)
            end
            ylabel('M', 'interpreter', 'latex','fontsize', 15)
        end
        % ----------------------------------------------
        % ----------------------------------------------
        function plot_bias_calibration(obj)
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.XX(10:12,:), 'linewidth',2)
            legend({'$acc_x$','$acc_y$','$acc_z$'},'interpreter', 'latex','fontsize', 15)
            
            figure; hold on; grid on;
            plot(obj.update.time, obj.update.XX(13:15,:), 'linewidth',2)
            legend({'$w_x$','$w_y$','$w_z$'},'interpreter', 'latex','fontsize', 15)
        end
        % ----------------------------------------------
        % ----------------------------------------------
    end
end
