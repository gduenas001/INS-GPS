clear
close all
clc

Number_of_cases = 4;

seed= 2;
rng(seed);

for j = 1 : Number_of_cases

figure()

%if j==1
%    title('Number of particles = 20')
%elseif j==2
%    title('Number of particles = 40')
%else
%    title('Number of particles = 80')
%end

hold on

number_of_particles = 10*2^j;

sigma =1;
true_mean = 0;

Prior_State = normrnd(true_mean,sigma,[1,number_of_particles]);

sampled_mean = mean(Prior_State);
plot(sampled_mean(1),0,'ro','LineWidth',2)

particle_weights = inf * ones(number_of_particles,1);

for i = 1 : number_of_particles
    particle_weights(i) = exp(-0.5*((sampled_mean-Prior_State(i))/sigma*(sampled_mean-Prior_State(i))'));
end

weighted_mean = (Prior_State*particle_weights/sum(particle_weights))';
plot(weighted_mean,0,'co','LineWidth',2)

%weighted_covariance = zeros(2);
%for i = 1 : number_of_particles
%    weighted_covariance = weighted_covariance + ( (weighted_mean-Prior_State(i,:))'*(weighted_mean-Prior_State(i,:))*particle_weights(i) );
%end
%weighted_covariance = 2*weighted_covariance/sum(particle_weights)
%shifted_mean = [5;3];

%mean_shift = ones( size(Prior_State) );
%mean_shift(:,1) = mean_shift(:,1)*shifted_mean(1);
%mean_shift(:,2) = mean_shift(:,2)*shifted_mean(2);

%Prior_State_shifted = mean_shift + Prior_State;

%error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.394,'style','b-')
%error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.865,'style','b-')
%error_ellipse(sigma,[mean(Prior_State(:,1)),mean(Prior_State(:,2))],0.989,'style','b-')
%error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.394,'style','r-')
%error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.865,'style','r-')
%error_ellipse(sigma,[mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2))],0.989,'style','r-')

histogram(Prior_State)
%plot(Prior_State_shifted(:,1),Prior_State_shifted(:,2),'r*','LineWidth',1)

%plot(true_mean(1),true_mean(2),'o','Color',[0, 0.4470, 0.7410],'LineWidth',4)

legend('Sampled mean','Weighted mean')

title(['Number of particles = ', num2str(number_of_particles), ', RMS = ',num2str(rms(weighted_mean-sampled_mean))])

%plot(shifted_mean(1),shifted_mean(2),'o','Color',[0.8500, 0.3250, 0.0980],'LineWidth',4)
%plot(mean(Prior_State_shifted(:,1)),mean(Prior_State_shifted(:,2)),'ro','LineWidth',2)

%axis equal
grid on
xlim([-3,3]);

end

%figure()

%diff_sample_mean_and_weighted_mwan