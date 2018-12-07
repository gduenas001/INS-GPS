
classdef DataClass < handle
    properties
        pred
        update
        
        msmts
        landmarks
        
    end
    
    methods
        % ----------------------------------------------
        % ----------------------------------------------
        function obj= DataClass(num_readings)
            obj.pred= PredictionDataClass(num_readings);
            obj.update= UpdateDataClass(num_readings);
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
        function plot_map(obj, gps, num_readings, params)
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
    end
end






