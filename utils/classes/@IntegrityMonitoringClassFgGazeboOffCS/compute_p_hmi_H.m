
function p_hmi_H= compute_p_hmi_H(obj, alpha, fault_ind, params)

 
% build extraction matrix
if fault_ind == 0
    obj.compute_E_matrix( 0, params.m_F);
else
    obj.compute_E_matrix( fault_ind, params.m_F);
end

% Worst-case fault direction
f_M_dir= obj.E' / (obj.E * obj.M_M * obj.E') * obj.E * obj.A * obj.PX_M * alpha;
f_M_dir= f_M_dir / norm(f_M_dir); % normalize

% worst-case fault magnitude
fx_hat_dir= abs( (alpha' / obj.Gamma_fg) * obj.A' * f_M_dir );
M_dir= abs( f_M_dir' * obj.M_M * f_M_dir );

% save the interesting values for the optimization
obj.counter_H= obj.counter_H + 1;
obj.noncentral_dof{obj.counter_H}=  obj.n_M + obj.n_M_gps;
obj.f_dir_sig2{obj.counter_H}= (fx_hat_dir / obj.sigma_hat)^2;
obj.M_dir{obj.counter_H}= M_dir;

% p_hmi_H= 0;
% return




% check if we should start evaluating f mag at zero
if abs(obj.optimization_fn(...
        0, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps )) >= 1
    % worst-case fault magnitude
    f_mag_min= 0;
    f_mag_max= 5;
    f_mag_inc= 5;
    p_hmi_H_prev= -1;
    for k= 1:10
        [f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
            f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M + obj.n_M_gps ),...
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
    
    [f_M_mag_out_1, p_hmi_H_1]= fminbnd( @(f_M_mag) obj.optimization_fn(...
        f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
        -10, 10);
    p_hmi_H_1= -p_hmi_H_1;
    if p_hmi_H_1 < 1e-10 || abs(f_M_mag_out_1) > 8
        [f_M_mag_out_2, p_hmi_H_2]= fminbnd( @(f_M_mag) obj.optimization_fn(...
            f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
            -100, 100);
        p_hmi_H_2= -p_hmi_H_2;
        if p_hmi_H_2 < 1e-10 || abs(f_M_mag_out_2) > 80
            [f_M_mag_out_3, p_hmi_H_3]= fminbnd( @(f_M_mag) obj.optimization_fn(...
                f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
                -1000, 1000);
            p_hmi_H_3= -p_hmi_H_3;
            if p_hmi_H_3 < 1e-10 || abs(f_M_mag_out_3) > 800
                [f_M_mag_out_4, p_hmi_H_4]= fminbnd( @(f_M_mag) obj.optimization_fn(...
                    f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
                    -100000, 100000);
                p_hmi_H_4= -p_hmi_H_4;
            end
        end
    end
    [p_hmi_H,ind]= max( [p_hmi_H_1, p_hmi_H_2, p_hmi_H_3, p_hmi_H_4] );
    if ind==1
        obj.f_M_mag_out= abs(f_M_mag_out_1);
    elseif ind==2
        obj.f_M_mag_out= abs(f_M_mag_out_2);
    elseif ind==3
        obj.f_M_mag_out= abs(f_M_mag_out_3);
    else
        obj.f_M_mag_out= abs(f_M_mag_out_4);
    end
end
% worst-case fault magnitude
%f_mag_min= 0;
%f_mag_max= 5;
%f_mag_inc= 5;
%p_hmi_H_prev= -1;

%for k=1:10
%    [obj.f_M_mag_out, p_hmi_H]= fminbnd( @(f_M_mag) obj.optimization_fn(...
%        f_M_mag, fx_hat_dir, M_dir, obj.sigma_hat, params.alert_limit, obj.n_M ),...
%        f_mag_min, f_mag_max);
%
%    % make it a positive number
%    p_hmi_H= -p_hmi_H;

    % check if the new P(HMI|H) is smaller
%    if k == 1 || p_hmi_H_prev < p_hmi_H
%       p_hmi_H_prev= p_hmi_H;
%       f_mag_min= f_mag_min + f_mag_inc;
%       f_mag_max= f_mag_max + f_mag_inc;
%    else
%       p_hmi_H= p_hmi_H_prev;
%       break
%    end
% end


end
