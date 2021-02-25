clear
close all
clc

seed= 2;
rng(seed);

number_of_particles = 1000;

figure()

hold on

prior_sigma =0.1;
prior_true_mean = 0;
prior_domain_min = -2;
prior_domain_max = 2;
prior_domain_size = 100000;

prior_samples = normrnd(prior_true_mean,prior_sigma,[1,number_of_particles]);

prior_sampled_mean = mean(prior_samples);
subplot(2,1,1)
histogram(prior_samples,100)
xlim([prior_domain_min/4,prior_domain_max/4])
grid on

prior_domain = linspace(prior_domain_min,prior_domain_max,number_of_particles);
prior_pdf = inf* ones( size(prior_domain) );
for i = 1 : length(prior_domain)
    prior_pdf(i) = normpdf(prior_domain(i),prior_true_mean,prior_sigma);
end
subplot(2,1,2)
plot(prior_domain,prior_pdf)
xlim([prior_domain_min/4,prior_domain_max/4])
xlabel('$\hat{x}_{k-1}$','interpreter', 'latex')
grid on

figure()

hold on

control_sigma =0.2;
control_true_mean = 1;
control_domain_min = 0;
control_domain_max = 2;
control_domain_size = 100000;

control_samples = normrnd(control_true_mean,control_sigma,[1,number_of_particles]);

control_sampled_mean = mean(control_samples);
subplot(2,1,1)
histogram(control_samples,100)
xlim([control_domain_min,control_domain_max])
grid on

control_domain = linspace(control_domain_min,control_domain_max,control_domain_size);
control_pdf = inf* ones( size(control_domain) );
for i = 1 : length(control_domain)
    control_pdf(i) = normpdf(control_domain(i),control_true_mean,control_sigma);
end
subplot(2,1,2)
plot(control_domain,control_pdf)
xlim([control_domain_min,control_domain_max])
xlabel('$u_{k-1}$','interpreter', 'latex')
grid on

figure()

hold on

subplot(2,1,1)
histogram(prior_samples,100)
xlim([prior_domain_min,prior_domain_max])
grid on
xlabel('$\hat{x}_{k-1}$','interpreter', 'latex')

timestep = 1;

pred_samples = inf* ones( size(prior_samples) );
for i = 1 : number_of_particles
    pred_samples(i) = prior_samples(i) + timestep*control_samples(i);
end
    
pred_sampled_mean = mean(pred_samples);
subplot(2,1,2)
histogram(pred_samples,100)
xlim([prior_domain_min,prior_domain_max])
grid on
xlabel('$\bar{x}_{k}$','interpreter', 'latex')

figure()

hold on

msmt = 0.5;
msmt_sigma = 0.15;
subplot(3,1,1)
histogram(pred_samples,100)
xlim([prior_domain_min,prior_domain_max])
grid on
xlabel('$\bar{x}_{k}$','interpreter', 'latex')

weight_domain = inf* ones( size(prior_domain) );
for i = 1 : length(prior_domain)
    weight_domain(i) = normpdf(prior_domain(i),msmt,msmt_sigma);
end
subplot(3,1,2)
plot(prior_domain,weight_domain)
xlim([prior_domain_min,prior_domain_max])
xlabel('$p(z_{k}|x_{k})$','interpreter', 'latex')
grid on

pred_weight = inf* ones( size(pred_samples) );
for i = 1 : length(pred_samples)
    pred_weight(i) = normpdf(pred_samples(i),msmt,msmt_sigma);
end
pred_weight= pred_weight/sum(pred_weight);

update_samples = inf* ones( size(pred_samples) );
for i = 1 : length(pred_samples)
    update_samples(i) = pred_samples( find( mnrnd(1,pred_weight) ) );
end
subplot(3,1,3)
histogram(update_samples,100)
xlim([prior_domain_min,prior_domain_max])
grid on
xlabel('$\hat{x}_{k}$','interpreter', 'latex')

figure()

hold on

prior_sigma =0.1;
prior_true_mean = 0;
prior_fault_min = -1;
prior_fault_max = 0.5;
prior_domain_min = -2;
prior_domain_max = 2;
prior_domain_size = 100000;

subplot(3,1,1)
histogram(prior_samples,100)
xlim([prior_domain_min,prior_domain_max])
xlabel('Unfaulted case')
grid on

subplot(3,1,2)
histogram(prior_samples+prior_fault_min,100)
xlim([prior_domain_min,prior_domain_max])
xlabel('faulted case')
grid on

subplot(3,1,3)
histogram(prior_samples+prior_fault_max,100)
xlim([prior_domain_min,prior_domain_max])
xlabel('faulted case')
grid on