clear
close all
clc

n_k = 20;
N= 203;
m=3;
lambda1=0;
lambda2=5;
lambda3=10;














x = (0.01:0.01:5.01)';
p1 = ncfpdf(x,n_k,N-m,lambda1);
p2 = ncfpdf(x,n_k,N-m,lambda2);
p3 = ncfpdf(x,n_k,N-m,lambda3);

figure;
plot(x,p1,'r-')
hold on
plot(x,p2,'g-')
plot(x,p3,'b-')
legend('\lambda = 0','\lambda = 5','\lambda = 10')
grid on
xlim([0,5])
title('Non-central Snedecors F PDF with 20 and 200 DoFs')
xlabel('q_k')
ylabel('PDF')



x = (0.01:0.01:5.01)';
p1 = ncfpdf(x,n_k,50,lambda3);
p2 = ncfpdf(x,n_k,100, lambda3);
p3 = ncfpdf(x,n_k,200, lambda3);

figure;
plot(x,p1,'r-')
hold on
plot(x,p2,'g-')
plot(x,p3,'b-')
legend('2^{nd} DoF = 50','2^{nd} DoF = 100','2^{nd} DoF = 200')
grid on
xlim([0,5])
title('Non-central Snedecors F PDF with \lambda = 10 and 20 1^{st} DoF')
xlabel('q_k')
ylabel('PDF')



x = (0.01:0.01:5.01)';
p1 = ncfpdf(x,5,N-m,lambda3);
p2 = ncfpdf(x,10,N-m, lambda3);
p3 = ncfpdf(x,20,N-m, lambda3);

figure;
plot(x,p1,'r-')
hold on
plot(x,p2,'g-')
plot(x,p3,'b-')
legend('1^{st} DoF = 5','1^{st} DoF = 10','1^{st} DoF = 20')
grid on
xlim([0,5])
title('Non-central Snedecors F PDF with \lambda = 10 and 200 2^{nd} DoF')
xlabel('q_k')
ylabel('PDF')