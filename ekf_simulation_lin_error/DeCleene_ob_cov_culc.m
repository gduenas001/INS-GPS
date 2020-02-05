% This function should be embedded into LinErrClass.
% Domain should follow the requirement in LinErrClass.
% Need to consider the case more than 3 dof.

clear; format short e; clc; close all;
dbstop if error


n = 1;
div = 1e-3;
x_range = 3000;
x_norm = -x_range:0.001:x_range;
x_chi2 = 0:0.001:x_range;
overbound = 0;
sigma_ob = 8.2;
    
while overbound == 0
    cdf_chi2 = chi2cdf(x_chi2,n)/2 + 1/2;
        
    %cdf_norm = cdf('Normal', x_norm, 0, sigma_ob);
    cdf_norm = cdf('Normal', x_chi2, 0, sigma_ob);
    if sum(cdf_norm<=cdf_chi2) == length(cdf_chi2)
      break;
    end
    sigma_ob = sigma_ob + div;
end

aug_cdf_chi2 = [fliplr(-cdf_chi2(1:end-1) + 1) 1/2 cdf_chi2(1:end-1)];
aug_cdf_norm = cdf('Normal',x_norm,0,sigma_ob);
p_chi2 = pdf('Chisquare',x_chi2,n);
p_norm = pdf('Normal',x_norm,0,sigma_ob);

disp("n:" + num2str(n) + ", var:" + sigma_ob^2);

figure(n)
hold on
%plot(x_norm,cdf_norm,'Color','r');
%plot(x_chi2,cdf_chi2,'Color','b');
plot(x_norm,aug_cdf_norm,'Color','r');
plot(x_norm,aug_cdf_chi2,'Color','b');
hold off

%figure(2*n)
%hold on
%plot(x_norm,p_norm,'Color','r');
%plot(x_chi2,p_chi2,'Color','b');
%plot(fliplr(-x_chi2),fliplr(p_chi2),'Color','b');
%hold off