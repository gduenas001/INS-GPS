function compute_required_epochs_for_min_LMs(obj, params, estimator)

obj.n_M= estimator.n_k;
i= 0; % initialize i to zero to indicate the current epoch
if sum(obj.n_ph) ~= 0
    for i= 1:length(obj.n_ph)
        obj.n_M= obj.n_M + obj.n_ph(i);
        % if the preceding horizon is long enough --> stop
        if ((obj.n_M/params.m_F) + ((estimator.n_gps_k + sum(obj.n_gps_ph(1:i)))/6)  >= params.min_n_L_M )...
                && (( (estimator.n_gps_k + sum(obj.n_gps_ph(1:i)) ~= 0) && params.SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt )...
                || ( ~params.SWITCH_FIXED_ABS_MSMT_PH_WITH_min_GPS_msmt )), break, end
    end
end
% set the variables
obj.n_L_M= obj.n_M / params.m_F;
estimator.n_L_M= obj.n_L_M;
obj.M= i + 1; % preceding epochs plus the current

end