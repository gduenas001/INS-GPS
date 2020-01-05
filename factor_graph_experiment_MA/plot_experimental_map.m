clear all;% close all; clc;

I_REQ= 1e-7;

% load data
load('fg_M30_nogps');

% figure
figure; hold on;


% ------------------------- alert limit of 0.5m -------------------------
subplot(2,1,1); hold on; box on;

% plot map
lm_map= [estimator.landmark_map(:,1), estimator.landmark_map(:,2)];
plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize', 10);

% plot path
plot(data_obj.update.x_true(1,1:end-1), data_obj.update.x_true(2,1:end-1), 'b-', 'linewidth', 2);


% non availabilty indexes
inds= data_obj.im.p_hmi > I_REQ;
time_naval= data_obj.update.time(inds);
poses_naval= data_obj.update.x_true(1:2,inds);

% report the availability
fprintf('Availability: %d\n', round(sum(data_obj.im.p_hmi < I_REQ) / length(data_obj.im.p_hmi)*100)  );


% find the bands where availability is not guarranteed to plot in red
bands_poses= cell(1,1000);
bands_times= cell(1,1000);
bands_ind= 1;
for i= 1:length(time_naval)
    if isempty( bands_times{bands_ind} )
        bands_times{bands_ind}= time_naval(i);
        bands_poses{bands_ind}= poses_naval(:,i);
    else
        if time_naval(i) - bands_times{bands_ind}(end) < 0.15
            bands_times{bands_ind}= [bands_times{bands_ind}, time_naval(i)];
            bands_poses{bands_ind}= [bands_poses{bands_ind}, poses_naval(:,i)];
        else
            bands_ind= bands_ind + 1;
        end
    end
    
end

% remove extra allocated memory
bands_times(bands_ind+1:end)= [];
bands_poses(bands_ind+1:end)= [];

% plot the bands in red
for i= 1:length(bands_times)
    plot(bands_poses{i}(1,:), bands_poses{i}(2,:), 'r.', 'linewidth', 2);
end



% axis limits
axis equal
x_limits= xlim;
y_limits= ylim;
set(gca,'xticklabel',[])


% axis labels
% xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% ---------------------------------------------------------------------

% ------------------------- alert limit of 1m -------------------------
load('fg_M30_l1');

subplot(2,1,2); hold on; box on;

% plot map
lm_map= [estimator.landmark_map(:,1), estimator.landmark_map(:,2)];
plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize', 10);

% plot path
plot(data_obj.update.x_true(1,1:end-1), data_obj.update.x_true(2,1:end-1), 'b-', 'linewidth', 2);


% non availabilty indexes
inds= data_obj.im.p_hmi > I_REQ;
time_naval= data_obj.update.time(inds);
poses_naval= data_obj.update.x_true(1:2,inds);

% report availability
fprintf('Availability: %d\n', round(sum(data_obj.im.p_hmi < I_REQ) / length(data_obj.im.p_hmi)*100)  );

% find the bands where availability is not guarranteed to plot in red
bands_poses= cell(1,1000);
bands_times= cell(1,1000);
bands_ind= 1;
for i= 1:length(time_naval)
    if isempty( bands_times{bands_ind} )
        bands_times{bands_ind}= time_naval(i);
        bands_poses{bands_ind}= poses_naval(:,i);
    else
        if time_naval(i) - bands_times{bands_ind}(end) < 0.15
            bands_times{bands_ind}= [bands_times{bands_ind}, time_naval(i)];
            bands_poses{bands_ind}= [bands_poses{bands_ind}, poses_naval(:,i)];
        else
            bands_ind= bands_ind + 1;
        end
    end
    
end

% remove extra allocated memory
bands_times(bands_ind+1:end)= [];
bands_poses(bands_ind+1:end)= [];

% plot the bands in red
for i= 1:length(bands_times)
    if ~isempty(bands_poses{i})
        plot(bands_poses{i}(1,:), bands_poses{i}(2,:), 'r.', 'linewidth', 2);
    end
end

% axis limits
axis equal
xlim(x_limits);
ylim(y_limits);
% ylim(y_limits+70)


% axis labels
xlabel('x [m]','interpreter', 'latex','fontsize', 10);
ylabel('y [m]','interpreter', 'latex','fontsize', 10);
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
% ---------------------------------------------------------------------


% % save figure
% fig= gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 3.5 5];
% print('path_exp_aval','-dpdf','-r0')
 

