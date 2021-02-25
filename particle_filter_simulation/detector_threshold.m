clear
close all
clc

n_k = 20;
N= 203;
m=3;
lambda1=0;
lambda2=3;














x = (0.01:0.01:5.01)';
p1 = ncfpdf(x,n_k,N-m,lambda1);
p2 = ncfpdf(x,n_k,N-m,lambda2);

y= 0:0.01:1.3;

figure;
plot(x,p1,'r-')
hold on
plot(x,p2,'b-')
plot(zeros(size(y))+1.8,y,'c-')
legend('\lambda = 0','\lambda = 3', 'T')
grid on
xlim([0,5])
ylim([0,1.3])
title('Non-central Snedecors F PDF with 20 and 200 DoFs')
xlabel('q_k')
ylabel('PDF')