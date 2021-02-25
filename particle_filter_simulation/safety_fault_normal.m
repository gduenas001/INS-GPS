clear
close all
clc

figure()
hold on
mu =0;

sigma = 1;

x=-5:0.01:5;

p_x_non_faulted = inf*ones(size(x));

for i =1 : length(x)
    p_x_non_faulted(i) = normpdf(x(i),mu,sigma);
end

plot(x,p_x_non_faulted,'b')
%y=0:0.01:0.4;
%plot(-2*ones(size(y)),y,'k')
%plot(2*ones(size(y)),y,'k')
%title('Non-faulted case')
%grid on

%figure()
%hold on
mu =2;


p_x_faulted = inf*ones(size(x));

for i =1 : length(x)
    p_x_faulted(i) = normpdf(x(i),mu,sigma);
end

plot(x,p_x_faulted,'r')

y=0:0.01:0.4;
plot(-2*ones(size(y)),y,'k')
plot(2*ones(size(y)),y,'k')
%title('Faulted case')
legend('Non-faulted case','Faulted case','2\sigma bound')
xlabel('\epsilon')
ylabel('PDF')
grid on
