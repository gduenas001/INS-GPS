
clear all; close all; clc;

% figure
figure; hold on; grid on;


for i= 1:2
    % select map ID
    if i == 1
        density= 1;
        map_i= 3;
    else 
        density= 5;
        map_i= 1;
    end
    
    % load data
    file_name_map= strcat('../data/simulation/factor_graph/results/density_00',...
        num2str(density), '_fix/map_', num2str(map_i), '/offline.mat' );
    load(file_name_map);
    
    % plot path
    subplot(2,1,i); hold on; box on;
    plot(data_obj.update.x_true(1,1:end-1), data_obj.update.x_true(2,1:end-1),...
        '-', 'linewidth', 2);
    
    % create a map of landmarks
    subplot(2,1,i); hold on;
    lm_map= [estimator.landmark_map(:,1),...
        estimator.landmark_map(:,2)];
    plot(lm_map(:,1), lm_map(:,2), 'k.', 'markersize', 15);
    
    % plot waypoints
    subplot(2,1,i); hold on;
    plot(params.way_points(1,:), params.way_points(2,:),...
        'gp', 'markersize', 10, 'MarkerFaceColor', 'g');
    
    % axis limits
    axis equal
    
    % move the x 20m to the right
    if i == 1
        x_limits= xlim;
        y_limits= ylim;
        set(gca,'XTickLabel',[])
    end
    xlim(x_limits + 20)
    xlim([-21.5, 220])
    ylim(y_limits)
    
    % axis labels
    if i == 2
        xlabel('x [m]','interpreter', 'latex','fontsize', 10);
    end
    ylabel('y [m]','interpreter', 'latex','fontsize', 10);
    set(gca,'TickLabelInterpreter','latex','fontsize', 10)
    
end


% save figure
fig= gcf;
fig.PaperUnits = 'inches';
fig.PaperPosition = [0 0 3.5 3.5];
file_name= 'maps';
print(file_name,'-dpdf','-r0')

