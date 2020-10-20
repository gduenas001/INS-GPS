clear
close all
clc

figure()
hold on

seed= 2;
rng(seed);

number_of_particles = 20;

rotation = [cos(pi/4),-sin(pi/4);sin(pi/4),cos(pi/4)];
sigma =rotation*[1,0;0,0.1]*rotation';
true_mean = [0;0];

Prior_State = mvnrnd(true_mean,sigma,number_of_particles);

shifted_mean = [5;3];

mean_shift = ones( size(Prior_State) );
mean_shift(:,1) = mean_shift(:,1)*shifted_mean(1);
mean_shift(:,2) = mean_shift(:,2)*shifted_mean(2);

Prior_State_shifted = mean_shift + Prior_State;

error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.394,'style','b-')
error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.865,'style','b-')
error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.989,'style','b-')
error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.394,'style','r-')
error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.865,'style','r-')
error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.989,'style','r-')

plot(Prior_State(:,1),Prior_State(:,2),'b*','LineWidth',1)
plot(Prior_State_shifted(:,1),Prior_State_shifted(:,2),'r*','LineWidth',1)

%plot(true_mean(1),true_mean(2),'o','Color',[0, 0.4470, 0.7410],'LineWidth',4)
plot(mean(Prior_State(:,1)),mean(Prior_State(:,2)),'bo','LineWidth',2)

%plot(shifted_mean(1),shifted_mean(2),'o','Color',[0.8500, 0.3250, 0.0980],'LineWidth',4)
plot(mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2)),'ro','LineWidth',2)

axis equal
grid on