clear
close all
clc

fault = 1;
sigma = 1
alert_limit = 1;
Num_of_particles=20;

sample_sigma = sqrt(var(normrnd(fault,sigma,[1,Num_of_particles])))


P_error_geq_alert_normal = normcdf((-alert_limit-fault)/( sigma/sqrt(Num_of_particles) )) + normcdf((alert_limit-fault)/( sigma/sqrt(Num_of_particles) ),'upper');

P_error_geq_alert_student_t = nctcdf(0,Num_of_particles-1,-(-alert_limit-fault)/( sigma/sqrt(Num_of_particles) )) + nctcdf(0,Num_of_particles-1,-(alert_limit-fault)/( sigma/sqrt(Num_of_particles) ),'upper');

P_error_geq_alert_student_t_wrong = nctcdf(-alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) )) + nctcdf(alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) ),'upper')

sigma = 10

P_error_geq_alert_student_t_wrong = nctcdf(-alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) )) + nctcdf(alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) ),'upper')

sigma = 0.5

P_error_geq_alert_student_t_wrong = nctcdf(-alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) )) + nctcdf(alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) ),'upper')

sigma = sample_sigma

P_error_geq_alert_student_t_wrong = nctcdf(-alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) )) + nctcdf(alert_limit/( sample_sigma/sqrt(Num_of_particles) ),Num_of_particles-1,fault/( sigma/sqrt(Num_of_particles) ),'upper')
