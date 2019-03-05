
clear all; clc; close all;

% distribution paramters
dof= 1;
ncp= 4;

% find the upper limit
x_val_max= ncx2inv(0.99, dof, ncp);

% set of values to check approximation
x_val= 0:0.01:x_val_max;


for i= 1:length(x_val)

    % exact CDF
    exact(i)= ncx2cdf(x_val(i), dof, ncp);
    
    % approximation
    r= sqrt(x_val(i));
    rho= sqrt(ncp);
    p= dof;
    z= r - rho - ((p-1)/2) * (log(r) - log(rho))/(r-rho);
    approx(i)= normcdf(z);
end


% fprintf( 'The difference between them is %f\n', abs(exact - approx) );


figure; hold on; grid on;
plot(x_val, exact)
plot(x_val, approx)
