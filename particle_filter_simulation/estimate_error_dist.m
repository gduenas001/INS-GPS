clear
close all
clc

N= 21;
mu_1=0;
mu_2=3;
mu_3=-3;














x = (-8.01:0.01:8.01)';
p1 = nctpdf(x,N-1,mu_1);
p2 = nctpdf(x,N-1,mu_2);
p3 = nctpdf(x,N-1,mu_3);

figure;
plot(x,p1,'r-')
hold on
plot(x,p2,'b-')
plot(x,p3,'g-')
legend('\mu = 0','\mu = 3', '\mu = -3')
grid on
xlim([-8,8])
%ylim([0,1.3])
title('Non-central students t PDF with 20 DoF')
xlabel('$\delta\bar{\hat{x}}$','interpreter', 'latex')
ylabel('PDF')


q1 = nctpdf(x,6-1,mu_2);
q2 = nctpdf(x,11-1,mu_2);
q3 = nctpdf(x,16-1,mu_2);


figure;
plot(x,q1,'r-')
hold on
plot(x,q2,'b-')
plot(x,q3,'g-')
legend('DoF = 5','Dof = 10', 'DoF = 15')
grid on
xlim([0,8])
%ylim([0,1.3])
title('Non-central students t PDF with \mu = 3')
xlabel('$\delta\bar{\hat{x}}$','interpreter', 'latex')
ylabel('PDF')