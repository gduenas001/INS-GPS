
clear; close all; clc;


pma0= load('P_MA_zero_no_MA');
pma1= load('P_MA_real');
ma0= load('no_MA');

%% plot integrity risk
figure; hold on; grid on;
plot(pma0.data_obj.im.time * pma0.params.velocity_sim, pma0.data_obj.im.p_hmi, 'linewidth', 2.3)
plot(ma0.data_obj.im.time * ma0.params.velocity_sim, ma0.data_obj.im.p_hmi, 'linewidth', 1.8)

plot(ma0.data_obj.im.time * ma0.params.velocity_sim,...
    ones(length(ma0.data_obj.im.time),1) * ma0.params.I_H, 'linewidth', 2)
set(gca,'TickLabelInterpreter','latex','fontsize', 12)

legend({'Assuming no MA','Actual integrity risk'}, 'location', 'southwest',...
    'interpreter', 'latex','fontsize', 12)
xlabel('x [m]','interpreter', 'latex','fontsize', 12)
xlim([0,116])
ylabel('$P(HMI)$','interpreter', 'latex','fontsize', 12)
set(gca, 'YScale', 'log')
xlim([0,116])
ylim([1e-20,1]);

% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 7 4];
% print('P_HMI','-dpdf','-r0')
% 

%% plot map and path

% set predictin and update in the same vector
pma1.poses= zeros(2, 2*length(pma1.data_obj.pred.time));
pma0.poses= zeros(2, 2*length(pma0.data_obj.pred.time));
for i= 1:length(pma0.data_obj.pred.time)
    ind_1= 2*i-1;
    ind_2= 2*i;
    pma1.poses(:,ind_1)= pma1.data_obj.pred.XX(1:2,i);
    pma1.poses(:,ind_2)= pma1.data_obj.update.XX(1:2,i);
    pma0.poses(:,ind_1)= pma0.data_obj.pred.XX(1:2,i);
    pma0.poses(:,ind_2)= pma0.data_obj.update.XX(1:2,i);
end

% create a map of landmarks
lm_map= [pma0.estimator.landmark_map(:,1),...
         pma0.estimator.landmark_map(:,2),...
         zeros(pma0.estimator.num_landmarks,1)];

% plots
figure; hold on; box on;
plot( pma0.poses(1,:), pma0.poses(2,:), '-', 'linewidth', 2.3);
plot( pma1.poses(1,:), pma1.poses(2,:), '-', 'linewidth', 1.7);
plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize',10, 'linewidth', 1);
set(gca,'TickLabelInterpreter','latex','fontsize', 12)
legend({'Without injected misassociation','With injected misassociation'},...
    'interpreter', 'latex','fontsize', 12)


% set axis
xlabel('x [m]','interpreter', 'latex','fontsize', 12);
ylabel('y [m]','interpreter', 'latex','fontsize', 12);
xlim([0,120])
axis equal

% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 7 4];
% print('path','-dpdf','-r0')

% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 4];
print('path_zoom','-dpdf','-r0')


%% plot error and variance envelope
pma0.standard_dev_y= sqrt( pma0.data_obj.update.PX(2,:) );
ma0.standard_dev_y= sqrt( pma0.data_obj.update.PX(2,:) );

figure; hold on; grid on;

plot(ma0.data_obj.update.time * ma0.params.velocity_sim,  ma0.data_obj.update.error(2,:), 'linewidth', 2)
plot(pma1.data_obj.update.time * pma1.params.velocity_sim, pma1.data_obj.update.error(2,:), 'linewidth', 2)


plot(pma0.data_obj.update.time * pma0.params.velocity_sim, 3*pma0.standard_dev_y,'k--','linewidth',2);
plot(pma0.data_obj.update.time * pma0.params.velocity_sim, -3*pma0.standard_dev_y,'k--','linewidth',2);
plot(pma0.data_obj.update.time * pma0.params.velocity_sim,...
    ones(length(pma0.data_obj.update.time),1)*0.9, '.', 'linewidth',2)
plot(pma0.data_obj.update.time * pma0.params.velocity_sim,...
    ones(length(pma0.data_obj.update.time),1)*(-0.9), '.', 'linewidth',2)

set(gca,'TickLabelInterpreter','latex','fontsize', 12)

legend({'$\delta \hat{x}$ without injected faults',...
        '$\delta \hat{x}$ with injected misassociation',...
        '$3 \hat{\sigma}$ variance envolope'},...
        'location', 'southwest',...
        'interpreter', 'latex','fontsize', 12)
xlabel('x [m]','interpreter', 'latex','fontsize', 12)
ylabel('Error [m]','interpreter', 'latex','fontsize', 12)
xlim([0,116])
ylim([-1.2, 1.2])

% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 4];
print('error','-dpdf','-r0')

%% plot detector and detector threshold

figure; hold on; grid on;
plot(pma0.data_obj.im.time * pma0.params.velocity_sim, pma0.data_obj.im.detector_threshold, 'linewidth', 2)
plot(pma0.data_obj.im.time * pma0.params.velocity_sim, pma0.data_obj.im.detector, 'linewidth', 2)

set(gca,'TickLabelInterpreter','latex','fontsize', 12)
xlabel('x [m]','interpreter', 'latex','fontsize', 12)
xlim([0,116])
ylim([0, 70])

legend({'Detector threshold, $T^q$',...
        'Fault detector, $q$'},...
        'location', 'southeast',...
        'interpreter', 'latex','fontsize', 12)

save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 7 4];
print('detector','-dpdf','-r0')
 


% %% plot miss-association probability by time
% 
% figure; hold on; grid on;
% for lm_id= 1:ma0.estimator.num_landmarks
%     lm{lm_id}= [];
%     for i= 1:length(ma0.data_obj.im.time)
%         % if it's empty --> continue
%         if isempty(ma0.data_obj.im.association), continue, end
%         
%         % if the landmark is associated
%         ind= find(ma0.data_obj.im.association{i} == lm_id);
%         if ~isempty(ind)
%             P_MA= ma0.data_obj.im.P_MA_k{i}(ind(1));
%             lm{lm_id}= [lm{lm_id};...
%                 ma0.data_obj.update.time(i) * ma0.params.velocity_sim, P_MA];
%         end
%     end
%     
%     % plot landmark P(MA)
%     plot(lm{lm_id}(:,1), lm{lm_id}(:,2), '-', 'linewidth', 2)
%     
% end
% xlabel('x [m]')
% ylabel('P(MA)')

% %% plot miss-association probability by landmark
% 
% figure; hold on; grid on;
% for i= 1:length(ma0.data_obj.im.time)
%     % if it's empty --> continue
%     if isempty(ma0.data_obj.im.association), continue, end
%     
%     % take the landmark indexes
%     lm_inds= ma0.data_obj.im.association{i};
%     P_MA= ma0.data_obj.im.P_MA_k{i};
%     
%     % plot
%     for j= 1:length(lm_inds)
%         plot( lm_inds(j), P_MA(j), 'bo' )
%     end
% end
% xlabel('landmark ID')
% ylabel('P(MA)')







