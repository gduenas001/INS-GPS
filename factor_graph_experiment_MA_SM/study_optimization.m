
clear all; close all; clc;

load('interesting_variables.mat')
ncp1= cell2mat(im.M_dir);
ncp2= cell2mat(im.f_dir_sig2);
dof1= cell2mat(im.noncentral_dof);

T2= 0.5^2;

for i= 30:length(ncp1)
    fprintf('i --> %d\n', i)
    
    % compute threshold
    T1= chi2inv(1 - 1e-5, dof1(i));
    
    % worst-case fault magnitude
    p_hmi_max= 0;
    f_mag_min= 0;
    f_mag_max= 5;
    f_mag_inc= 5;
    p_hmi_H_prev= -1;
    for k= 1:10
        % test
        p_hmi_H= im.optimization_fn_test( (f_mag_min + f_mag_max)/2, ncp1(i), ncp2(i), dof1(i),  T1, T2)
        
        % optimize
        [f_mag_out, p_hmi_H]= fminbnd( @(f_mag) im.optimization_fn_test(...
            f_mag, ncp1(i), ncp2(i), dof1(i),  T1, T2),...
            f_mag_min, f_mag_max);
        
        % make it a positive number
        p_hmi_H= -p_hmi_H;
        
        % check if the new P(HMI|H) is smaller
        if k == 1 || p_hmi_H_prev < p_hmi_H || p_hmi_H < 1e-10
            p_hmi_H_prev= p_hmi_H;
            f_mag_min= f_mag_max;
            f_mag_max= f_mag_max* f_mag_inc;
            p_hmi_max= max(p_hmi_H, p_hmi_max);
        else
            p_hmi_max= max(p_hmi_H, p_hmi_max);
            break
        end
    end
    p_hmi_max
end