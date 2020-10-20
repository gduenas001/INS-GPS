clear; clc;

test_n5= load('test_n5_P_MA_max');
test_n10= load('test_n10_P_MA_max_1e_minus_3');
test_n15= load('test_n15_P_MA_max_1e_minus_2');

time_im_n5 = test_n5.data_obj.im.time;
time_update_n5 = test_n5.data_obj.update.time;
Ind_Equiv_time_update_n5 = inf*ones(size(time_im_n5));
for i = 1:length(Ind_Equiv_time_update_n5)
    [~,Ind_Equiv_time_update_n5(i)] = min(abs(time_update_n5-time_im_n5(i)));
end
Distance_im_test_n5 = inf*ones(size(Ind_Equiv_time_update_n5));
tmp=0;
for i = 1:length(Ind_Equiv_time_update_n5)
    if i==1
        Distance_im_test_n5(i) = tmp;
    else
        tmp = tmp+norm(test_n5.data_obj.update.XX(1:2,i)-test_n5.data_obj.update.XX(1:2,i-1));
        Distance_im_test_n5(i) = tmp;
    end
end

time_im_n10 = test_n10.data_obj.im.time;
time_update_n10 = test_n10.data_obj.update.time;
Ind_Equiv_time_update_n10 = inf*ones(size(time_im_n10));
for i = 1:length(Ind_Equiv_time_update_n10)
    [~,Ind_Equiv_time_update_n10(i)] = min(abs(time_update_n10-time_im_n10(i)));
end
Distance_im_test_n10 = inf*ones(size(Ind_Equiv_time_update_n10));
tmp=0;
for i = 1:length(Ind_Equiv_time_update_n10)
    if i==1
        Distance_im_test_n10(i) = tmp;
    else
        tmp = tmp+norm(test_n10.data_obj.update.XX(1:2,i)-test_n10.data_obj.update.XX(1:2,i-1));
        Distance_im_test_n10(i) = tmp;
    end
end

time_im_n15 = test_n15.data_obj.im.time;
time_update_n15 = test_n15.data_obj.update.time;
Ind_Equiv_time_update_n15 = inf*ones(size(time_im_n15));
for i = 1:length(Ind_Equiv_time_update_n15)
    [~,Ind_Equiv_time_update_n15(i)] = min(abs(time_update_n15-time_im_n15(i)));
end
Distance_im_test_n15 = inf*ones(size(Ind_Equiv_time_update_n15));
tmp=0;
for i = 1:length(Ind_Equiv_time_update_n15)
    if i==1
        Distance_im_test_n15(i) = tmp;
    else
        tmp = tmp+norm(test_n15.data_obj.update.XX(1:2,i)-test_n15.data_obj.update.XX(1:2,i-1));
        Distance_im_test_n15(i) = tmp;
    end
end
%% plot P(MA) for specific landmarks

% interesting landmarks (1-6,14-20, 35-27, 31-34, 115-116, 120-124

lm_ids= [91,90,94];
skip=inf*ones(1,3);
figure; hold on; grid on;
for j= 1:length(lm_ids)
    lm_id= lm_ids(j);
    
    time= [];
    Distance=[];
    P_MA= [];
    
%     for i= 1:length(test_n10.data_obj.im.time)
%         
%         % eliminate the non-associated bc of NN
%         association= test_n10.data_obj.im.association_full{i};
%         association( association == 0 )= [];
%         
%         % if it's empty --> continue
%         if isempty(association), continue, end
%         
%         ind= find(lm_id == association);
%         if ~isempty(ind)
%             time= [time; test_n10.data_obj.im.time(i)];
%             if length(association) == 1
%                 P_MA= [P_MA; 0];
%             else
%                 P_MA= [P_MA; test_n10.data_obj.im.P_MA_k_full{i}(ind)];
%             end
%         end
%     end


    for i= 1:length(Distance_im_test_n10)
        
        % eliminate the non-associated bc of NN
        association= test_n10.data_obj.im.association_full{i};
        association( association == 0 )= [];
        
        % if it's empty --> continue
        if isempty(association), continue, end
        
        ind= find(lm_id == association);
        if ~isempty(ind)
            Distance= [Distance; Distance_im_test_n10(i)];
            if length(association) == 1
                P_MA= [P_MA; 0];
            else
                P_MA= [P_MA; test_n10.data_obj.im.P_MA_k_full{i}(ind)];
            end
        end
    end

    
    %if ~isempty(time)
    %    plot(time, P_MA, 'linewidth', 2)
    %end
    if ~isempty(Distance)
        if j==1
            skip(1)=37;
            Distance_1=Distance;
            P_MA_1=P_MA;
        elseif j==2
            skip(2)=40;
            Distance_2=Distance;
            P_MA_2=P_MA;
        else
            skip(3)=29;
            Distance_3=Distance;
            P_MA_3=P_MA;
        end
        %plot(Distance, P_MA, 'linewidth', 2)
    end
end
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
plot(Distance_1(1:skip(1)), P_MA_1(1:skip(1)),'Color',[0.4940    0.1840    0.5560], 'linewidth', 2)
plot(Distance_2(1:skip(2)), P_MA_2(1:skip(2)),'Color',[0.4660    0.6740    0.1880], 'linewidth', 2)
plot(Distance_3(1:skip(3)), P_MA_3(1:skip(3)),'Color',[0.3010    0.7450    0.9330], 'linewidth', 2)
plot(Distance_1(skip(1)+1:end), P_MA_1(skip(1)+1:end),'Color',[0.4940    0.1840    0.5560], 'linewidth', 2)
plot(Distance_2(skip(2)+1:end), P_MA_2(skip(2)+1:end),'Color',[0.4660    0.6740    0.1880], 'linewidth', 2)
plot(Distance_3(skip(3)+1:end), P_MA_3(skip(3)+1:end),'Color',[0.3010    0.7450    0.9330], 'linewidth', 2)
%xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
leg= legend({'$LM1$', '$LM2$', '$LM3$'},'interpreter', 'latex','fontsize', 10);
xlabel('Distance travelled [m]','interpreter', 'latex','fontsize', 10)
%xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
xlim([Distance_im_test_n10(1), Distance_im_test_n10(end)]) % reset the x-axis (otherwise it moves)
ylabel('P(MA)','interpreter', 'latex','fontsize', 10)
%xlim([187.2, 204]) % rset the x-axis (otherwise it moves)
xlim([52, 80]) % rset the x-axis (otherwise it moves)
ylim([0.08,0.65]);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 2.5];
print('P_MA_test','-dpdf','-r0')
%% plot integrity risk
figure; hold on; grid on;
% plot(test_n5.data_obj.im.time, test_n5.data_obj.im.p_hmi + test_n10.params.I_H, 'g-', 'linewidth', 2)
% plot(test_n10.data_obj.im.time, test_n10.data_obj.im.p_hmi + test_n10.params.I_H, 'b-', 'linewidth', 2)
% plot(test_n15.data_obj.im.time, test_n15.data_obj.im.p_hmi + test_n10.params.I_H, 'r-', 'linewidth', 2)
plot(Distance_im_test_n5, test_n5.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 2)
plot(Distance_im_test_n10, test_n10.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 2)
plot(Distance_im_test_n15, test_n15.data_obj.im.p_hmi + test_n10.params.I_H, 'linewidth', 2)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)

leg= legend({'$n^{F^{(M)}} = 5$','$n^{F^{(M)}} = 10$','$n^{F^{(M)}} = 15$'}, 'interpreter', 'latex','fontsize', 10);
%xlabel('Time [s]','interpreter', 'latex','fontsize', 10)
xlabel('Distance travelled [m]','interpreter', 'latex','fontsize', 10)
%xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
xlim([Distance_im_test_n10(1), Distance_im_test_n10(end)]) % reset the x-axis (otherwise it moves)
ylabel('P(HMI)','interpreter', 'latex','fontsize', 10)
set(gca, 'YScale', 'log')
%xlim([test_n10.data_obj.im.time(1), test_n10.data_obj.im.time(end)]) % reset the x-axis (otherwise it moves)
%xlim([Distance_im_test_n10(1), Distance_im_test_n10(end)]) % reset the x-axis (otherwise it moves)
%xlim([0.002, 0.021]) % reset the x-axis (otherwise it moves)
ylim([7*1e-8,1]);


% save fig
fig= gcf;% fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 3];
print('P_HMI_test','-dpdf','-r0')

% % for the zoom in
% xlim([test_n10.data_obj.im.time(1), 104])
% set(leg, 'visible', 'off')
%  
% % save fig
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.5 1.5];
% print('P_HMI_zoom_test','-dpdf','-r0')

%% plot map and path for test_n10

figure; hold on; grid on;
plot3(test_n10.data_obj.pred.XX(1,:), test_n10.data_obj.pred.XX(2,:), test_n10.data_obj.pred.XX(3,:), 'b.');
plot3(test_n10.data_obj.update.XX(1,:), test_n10.data_obj.update.XX(2,:), test_n10.data_obj.update.XX(3,:),...
    'b.','markersize', 7);

% create a map of landmarks
lm_map= [test_n10.estimator.landmark_map(:,1),...
    test_n10.estimator.landmark_map(:,2),...
    zeros(test_n10.estimator.num_landmarks,1)];
plot3(lm_map(:,1), lm_map(:,2), lm_map(:,3), 'k.', 'markersize', 6);

% plot landmarks IDs
for i= 1:length(lm_map)
    txt= num2str(i);
    text(lm_map(i,1), lm_map(i,2), txt);
    
end

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis equal
xlim([-315, 20])
ylim([-95, 25])
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);

% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 2.5];
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

