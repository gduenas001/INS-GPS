clear
close all
clc















figure()
hold on

rng(1)

rotation = [cos(pi/4),-sin(pi/4);sin(pi/4),cos(pi/4)];
sigma =rotation*[1,0;0,0.1]*rotation';
true_mean = [2;2];

number_of_particles = 20;

particles = mvnrnd(true_mean,sigma,number_of_particles);

sigma = cov(particles);

plot(particles(:,1), particles(:,2),'b*')

error_ellipse(sigma,[mean(particles(:,1)),mean(particles(:,2))],0.865,'style','r-')

xlim([0,4])
ylim([0,4])

legend('Particles','2\sigma envelope')

xlabel('\epsilon_x')
ylabel('\epsilon_y')

grid on