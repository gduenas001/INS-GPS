clear; close all; clc;

test_n5= load('test_n5_P_MA_max');
test_n10= load('test_n10_P_MA_max_1e_minus_3');
test_n15= load('test_n15_P_MA_max_1e_minus_2');

%% plot P(MA) for specific landmarks

% interesting landmarks (1-6,14-20, 35-27, 31-34, 115-116, 120-124

lm_ids= [91,90,94];

figure; hold on; grid on;
for j= 1:length(lm_ids)
    lm_id= lm_ids(j);
    
    time= [];
    P_MA= [];
    
    for i= 1:length(test_n10.data_obj.im.time)
        
        % eliminate the non-associated bc of NN
        association= test_n10.data_obj.im.association_full{i};
        association( association == 0 )= [];
        
        % if it's empty --> continue
        if isempty(association), continue, end
        
        ind= find(lm_id == association);
        if ~isempty(ind)
            time= [time; test_n10.data_obj.im.time(i)];
            if length(association) == 1
                P_MA= [P_MA; 0];
            else
                P_MA= [P_MA; test_n10.data_obj.im.P_MA_k_full{i}(ind)];
            end
        end
    end
    
    if ~isempty(time)
        plot(time, P_MA, 'linewidth', 2)
    end
end

legend({'lm 1', 'lm 2', 'lm 3'},...
    'location', 'northwest',...
    'interpreter', 'latex','fontsize', 12);
xlabel('Time [s]','interpreter', 'latex','fontsize', 12)
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
ylabel('$P^{MA}$','interpreter', 'latex','fontsize', 12)
xlim([187.2, 204]) % rset the x-axis (otherwise it moves)
ylim([0.08,0.65]);
set(gca,'TickLabelInterpreter','latex','fontsize', 12)

% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 3.5];
print('P_MA_test','-dpdf','-r0')

%% plot integrity risk

figure; hold on; grid on;

plot(test_n5.data_obj.im.time, test_n5.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 1.5)
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 1.5)
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 1.5)

leg= legend({'Min. associations, $n^{A^{(M)}}_{min} = 5$',...
             'Min. associations, $n^{A^{(M)}}_{min} = 10$',...
             'Min. associations, $n^{A^{(M)}}_{min} = 15$'},...
            'interpreter', 'latex','fontsize', 12);

set(gca,'TickLabelInterpreter','latex','fontsize', 12)

xlabel('Time [s]','interpreter', 'latex','fontsize', 12)
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
ylabel('$P(HMI)$','interpreter', 'latex','fontsize', 12)
set(gca, 'YScale', 'log')
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
ylim([5*1e-8,1]);


% % save fig
% fig= gcf;% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 7 4];
% print('P_HMI_test','-dpdf','-r0')


% for the zoom in
xlim([test_n10.data_obj.im.time(1), 104])
set(leg, 'visible', 'off')
 
% save fig
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 2];
print('P_HMI_zoom_test','-dpdf','-r0')

%% plot map and path for test_n10

figure; hold on; box on;
plot3(test_n10.data_obj.pred.XX(1,:), test_n10.data_obj.pred.XX(2,:), test_n10.data_obj.pred.XX(3,:), '.');
% plot3(test_n10.data_obj.update.XX(1,:), test_n10.data_obj.update.XX(2,:), test_n10.data_obj.update.XX(3,:),...
%     '.','markersize', 7);

% create a map of landmarks
lm_map= [test_n10.estimator.landmark_map(:,1),...
    test_n10.estimator.landmark_map(:,2),...
    zeros(test_n10.estimator.num_landmarks,1)];
plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'k.', 'markersize', 12);

% % plot landmarks IDs
% for i= 1:length(lm_map)
%     txt= num2str(i);
%     text(lm_map(i,1), lm_map(i,2), txt);
%     
% end

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis equal
xlim([-315, 20])
ylim([-95, 25])
set(gca,'TickLabelInterpreter','latex','fontsize', 12)
xlabel('x [m]','interpreter', 'latex','fontsize', 12);
ylabel('y [m]','interpreter', 'latex','fontsize', 12);

% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 3];
print('path_test','-dpdf','-r0')



% plot3(test_n10.data_obj.msmts(:,1), test_n10.data_obj.msmts(:,2), zeros(size(test_n10.data_obj.msmts,1),1), 'k.');
% plot3(test_n10.gps.msmt(1,:),test_n10.gps.msmt(2,:),test_n10.gps.msmt(3,:),'r*');

% % plot attitude every 100 IMU readings
% for i= 1:test_n10.imu.num_readings
%     if rem(i,100) == 0
%         R_NB= R_NB_rot(test_n10.data_obj.pred.XX(7,i), test_n10.data_obj.pred.XX(8,i), test_n10.data_obj.pred.XX(9,i));
%         xyz_N= R_NB*test_n10.params.xyz_B + test_n10.data_obj.pred.XX(1:3,i);
%         plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
%     end
% end


%% plot detector and detector threshold for test_n10 & test_n15

figure; hold on; grid on;
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.detector, 'b-', 'linewidth', 2)
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.detector_threshold, 'b--', 'linewidth', 2)
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.detector, 'r-', 'linewidth', 2)
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.detector_threshold, 'r--', 'linewidth', 2)
plot(test_n5.data_obj.im.time, test_n5.data_obj.im.detector, 'g-', 'linewidth', 2)
plot(test_n5.data_obj.im.time, test_n5.data_obj.im.detector_threshold, 'g--', 'linewidth', 2)

set(gca,'TickLabelInterpreter','latex','fontsize', 10)
xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)])
ylim([0, 110])

legend({'$q_{D}$', '$T_D$'},'interpreter', 'latex','fontsize', 10)

% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.7 2.5];
% print('detector','-dpdf','-r0')

%% plot P_MA for test_n10
figure; hold on; grid on;
for i= 1:length(test_n10.data_obj.im.time)
    % if it's empty --> continue
    if isempty(test_n10.data_obj.im.association_full{i}), continue, end

    % take the landmark indexes
    lm_inds= test_n10.data_obj.im.association_full{i};
    P_MA_full= test_n10.data_obj.im.P_MA_k_full{i};
    lm_inds(lm_inds==0)= [];
    % plot
    if length(lm_inds) > 1
        for j= 1:length(lm_inds)
            plot( lm_inds(j), P_MA_full(j), 'bo' )
        end
    end
end
xlabel('landmark ID')
ylabel('P(MA)')


% %% plot P_MA for test_n15
% figure; hold on; grid on;
% for i= 1:length(test_n15.data_obj.im.time)
%     % if it's empty --> continue
%     if isempty(test_n15.data_obj.im.association_full), continue, end
% 
%     % take the landmark indexes
%     lm_inds= test_n15.data_obj.im.association_full{i};
%     P_MA_full= test_n15.data_obj.im.P_MA_k_full{i};
%     lm_inds(lm_inds==0)=[];
%     % plot
%     if length(lm_inds) > 1
%         for j= 1:length(lm_inds)
%             plot( lm_inds(j), P_MA_full(j), 'bo' )
%         end
%     end
% end
% xlabel('landmark ID')
% ylabel('P(MA)')
% % % save figure
% % fig= gcf;
% % fig.PaperUnits = 'inches';
% % fig.PaperPosition = [0 0 3.7 2.5];
% % print('path','-dpdf','-r0')
% % save figure
% % fig= gcf;
% % fig.PaperUnits = 'inches';
% % fig.PaperPosition = [0 0 3.7 1.2];
% % print('path_zoom','-dpdf','-r0')

