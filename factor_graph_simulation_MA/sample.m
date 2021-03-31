load('params.mat');
Lambda_k_M = A_k_M'*A_k_M;
f_h_dir = F_h' / ( F_h*( eye(N) - A_k_M/Lambda_k_M*A_k_M' )*F_h' ) *F_h*A_k_M/Lambda_k_M*alpha;
f_h_dir= f_h_dir / norm(f_h_dir);

mu_alphat_delta_k_dir = alpha' / Lambda_k_M * A_k_M' * f_h_dir;

lambda_k_dir = f_h_dir' * ( eye( N ) - A_k_M / Lambda_k_M * A_k_M' ) * f_h_dir;
        
        
if abs(optimization_fn(...
            0, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k )) > 1e-10
    % worst-case fault magnitude
    f_mag_min= 0;
    f_mag_max= 5;
    f_mag_inc= 5;
    p_hmi_H_prev= -1;
    for k= 1:10
        [f_h_mag_worst, p_hmi_H]= fminbnd( @(f_h_mag) optimization_fn(...
            f_h_mag, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k ),...
            f_mag_min, f_mag_max);
            
        % make it a positive number
        p_hmi_H= -p_hmi_H;
        
        % check if the new P(HMI|H) is smaller
        if k == 1 || p_hmi_H_prev < p_hmi_H
            p_hmi_H_prev= p_hmi_H;
            f_mag_min= f_mag_min + f_mag_inc;
            f_mag_max= f_mag_max + f_mag_inc;
        else
            p_hmi_H= p_hmi_H_prev;
            break
        end
    end
    % make a general optimization first
else
    p_hmi_H_1= 0;
    p_hmi_H_2= 0;
    p_hmi_H_3= 0;
    p_hmi_H_4= 0;
    f_M_mag_out_1= -10;
    f_M_mag_out_2= -10;
    f_M_mag_out_3= -10;
    f_M_mag_out_4= -10;
    
    [f_M_mag_out_1, p_hmi_H_1]= fminbnd( @(f_h_mag) optimization_fn(...
            f_h_mag, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k ),...
            -10, 10);
    p_hmi_H_1= -p_hmi_H_1;
    if p_hmi_H_1 < 1e-10 || abs(f_M_mag_out_1) > 8
        [f_M_mag_out_2, p_hmi_H_2]= fminbnd( @(f_h_mag) optimization_fn(...
            f_h_mag, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k ),...
            -100, 100);
        p_hmi_H_2= -p_hmi_H_2;
        if p_hmi_H_2 < 1e-10 || abs(f_M_mag_out_2) > 80
            [f_M_mag_out_3, p_hmi_H_3]= fminbnd( @(f_h_mag) optimization_fn(...
            f_h_mag, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k ),...
            -1000, 1000);
            p_hmi_H_3= -p_hmi_H_3;
            if p_hmi_H_3 < 1e-10 || abs(f_M_mag_out_3) > 800
                [f_M_mag_out_4, p_hmi_H_4]= fminbnd( @(f_h_mag) optimization_fn(...
            f_h_mag, mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k ),...
            -100000, 100000);
                p_hmi_H_4= -p_hmi_H_4;
            end
        end
    end
    [p_hmi_H,ind]= max( [p_hmi_H_1, p_hmi_H_2, p_hmi_H_3, p_hmi_H_4] );
end

if ind==1
    f_h_mag_worst= abs(f_M_mag_out_1);
elseif ind==2
    f_h_mag_worst= abs(f_M_mag_out_2);
elseif ind==3
    f_h_mag_worst= abs(f_M_mag_out_3);
else
    f_h_mag_worst= abs(f_M_mag_out_4);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the p_hmi as a function of the fault magnitude
figure();
grid on;
f_h_mag_plot=-100:0.5:100;
p_hmi_H_plot=zeros(size(f_h_mag_plot));
for i = 1:length(f_h_mag_plot)
    p_hmi_H_plot(i)= -optimization_fn(f_h_mag_plot(i), mu_alphat_delta_k_dir, lambda_k_dir, sqrt( alpha'/Lambda_k_M*alpha ), l, N - (M+1)*m, T_k );
end
plot(f_h_mag_plot,p_hmi_H_plot)
xlabel('f_{mag}')
ylabel('P(HMI_{k})')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%