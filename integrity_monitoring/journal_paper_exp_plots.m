clear; close all; clc;

test_n10= load('test_n10_P_MA_max_1e_minus_3');
test_n15= load('test_n15_P_MA_max_1e_minus_2');

%% plot integrity risk
figure; hold on; grid on;
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.p_hmi, 'b-', 'linewidth', 2)
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.p_hmi, 'r-', 'linewidth', 2)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)

legend({'preceding horizon with at least 10 landmarks','preceding horizon with at least 15 landmarks'}, 'interpreter', 'latex','fontsize', 10)
xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
ylabel('P(HMI)','interpreter', 'latex','fontsize', 10)
set(gca, 'YScale', 'log')
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
ylim([1e-15,1]);

% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.5 2.5];
% print('P_HMI','-dpdf','-r0')

%% plot map and path for test_n10

figure; hold on; grid on;
plot3(test_n10.data_obj.pred.XX(1,:), test_n10.data_obj.pred.XX(2,:), test_n10.data_obj.pred.XX(3,:), 'b.');
plot3(test_n10.data_obj.update.XX(1,:), test_n10.data_obj.update.XX(2,:), test_n10.data_obj.update.XX(3,:),...
    'b.','markersize', 7);
plot3(test_n10.gps.msmt(1,:),test_n10.gps.msmt(2,:),test_n10.gps.msmt(3,:),'r*');

% create a map of landmarks
lm_map= [test_n10.estimator.landmark_map(:,1),...
    test_n10.estimator.landmark_map(:,2),...
    zeros(test_n10.estimator.num_landmarks,1)];
plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'g+', 'markersize',20);
plot3(test_n10.data_obj.msmts(:,1), test_n10.data_obj.msmts(:,2), zeros(size(test_n10.data_obj.msmts,1),1), 'k.');

% plot attitude every 100 IMU readings
for i= 1:test_n10.imu.num_readings
    if rem(i,100) == 0
        R_NB= R_NB_rot(test_n10.data_obj.pred.XX(7,i), test_n10.data_obj.pred.XX(8,i), test_n10.data_obj.pred.XX(9,i));
        xyz_N= R_NB*params.xyz_B + test_n10.data_obj.pred.XX(1:3,i);
        plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
    end
end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis equal
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% legend({'With injected MA','Without injected MA'}, 'interpreter', 'latex','fontsize', 10)
% set axis
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);

%% plot map and path for test_n15
figure; hold on; grid on;
plot3(test_n15.data_obj.pred.XX(1,:), test_n15.data_obj.pred.XX(2,:), test_n15.data_obj.pred.XX(3,:), 'b.');
plot3(test_n15.data_obj.update.XX(1,:), test_n15.data_obj.update.XX(2,:), test_n15.data_obj.update.XX(3,:),...
    'b.','markersize', 7);
plot3(test_n15.gps.msmt(1,:),test_n15.gps.msmt(2,:),test_n15.gps.msmt(3,:),'r*');

% create a map of landmarks
lm_map= [test_n15.estimator.landmark_map(:,1),...
    test_n15.estimator.landmark_map(:,2),...
    zeros(test_n15.estimator.num_landmarks,1)];
plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'g+', 'markersize',20);
plot3(test_n15.data_obj.msmts(:,1), test_n15.data_obj.msmts(:,2), zeros(size(test_n15.data_obj.msmts,1),1), 'k.');

% plot attitude every 100 IMU readings
for i= 1:test_n15.imu.num_readings
    if rem(i,100) == 0
        R_NB= R_NB_rot(test_n15.data_obj.pred.XX(7,i), test_n15.data_obj.pred.XX(8,i), test_n15.data_obj.pred.XX(9,i));
        xyz_N= R_NB*params.xyz_B + test_n15.data_obj.pred.XX(1:3,i);
        plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
    end
end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis equal
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% legend({'With injected MA','Without injected MA'}, 'interpreter', 'latex','fontsize', 10)
% set axis
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);

% axis equal

%% plot detector and detector threshold for test_n10

figure; hold on; grid on;
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.detector, 'linewidth', 2)
plot(test_n10.data_obj.im.time, test_n10.data_obj.im.detector_threshold, 'linewidth', 2)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)])
ylim([0, 110])

legend({'$q_{D}$', '$T_D$'},'interpreter', 'latex','fontsize', 10)

fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.7 2.5];
print('detector','-dpdf','-r0')

%% plot detector and detector threshold for test_n15

figure; hold on; grid on;
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.detector, 'linewidth', 2)
plot(test_n15.data_obj.im.time, test_n15.data_obj.im.detector_threshold, 'linewidth', 2)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
xlim([test_n15.data_obj.im.time(1), test_n15.data_obj.im.time(end)])
ylim([0, 110])

legend({'$q_{D}$', '$T_D$'},'interpreter', 'latex','fontsize', 10)

fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.7 2.5];
print('detector','-dpdf','-r0')

%% plot P_MA for test_n10
figure; hold on; grid on;
for i= 1:length(test_n10.data_obj.im.time)
    % if it's empty --> continue
    if isempty(test_n10.data_obj.im.association_full), continue, end

    % take the landmark indexes
    lm_inds= test_n10.data_obj.im.association_full{i};
    P_MA_full= test_n10.data_obj.im.P_MA_k_full{i};
    lm_inds(lm_inds==0)=[];
    % plot
    if length(lm_inds) > 1
        for j= 1:length(lm_inds)
            plot( lm_inds(j), P_MA_full(j), 'bo' )
        end
    end
end
xlabel('landmark ID')
ylabel('P(MA)')

%% plot P_MA for test_n15
figure; hold on; grid on;
for i= 1:length(test_n15.data_obj.im.time)
    % if it's empty --> continue
    if isempty(test_n15.data_obj.im.association_full), continue, end

    % take the landmark indexes
    lm_inds= test_n15.data_obj.im.association_full{i};
    P_MA_full= test_n15.data_obj.im.P_MA_k_full{i};
    lm_inds(lm_inds==0)=[];
    % plot
    if length(lm_inds) > 1
        for j= 1:length(lm_inds)
            plot( lm_inds(j), P_MA_full(j), 'bo' )
        end
    end
end
xlabel('landmark ID')
ylabel('P(MA)')
% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.7 2.5];
% print('path','-dpdf','-r0')
% save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.7 1.2];
% print('path_zoom','-dpdf','-r0')