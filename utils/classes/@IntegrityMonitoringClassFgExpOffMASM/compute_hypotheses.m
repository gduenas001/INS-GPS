
function compute_hypotheses(obj, params)

% %n_max_blanch= obj.n_max;

% % probability of "r" or more simultaneous faults
% flag_out= false;
% for r= 1:length(obj.P_F_M)
%     if  sum(obj.P_F_M)^r  / factorial(r)  < params.I_H
%         obj.n_max= r-1;
%         flag_out= true;
%         break
%     end
% end
% 
% % if no "r" holds --> all landmarks failing simultaneously must be monitored
% if ~flag_out, obj.n_max= r; end
% 
% if obj.n_max > 1
%     fprintf('n_max: %d\n', obj.n_max);
%     if params.SWITCH_ONLY_ONE_LM_FAULT
%         obj.n_max= 1;
%     end
% end

% compute number of hypotheses
obj.n_H= 0;
obj.inds_H= cell(200,1);
start_ind= 1;
for num_faults= 1:obj.n_max
    
    if params.SWITCH_FACTOR_GRAPHS && (~params.SWITCH_SIM)
        
        new_H= nchoosek(obj.n_L_M + (obj.n_M_gps/6), num_faults);
        obj.n_H= obj.n_H + new_H;
        obj.inds_H( start_ind:start_ind+new_H - 1, 1 )=...
            num2cell( nchoosek(1:( obj.n_L_M + (obj.n_M_gps/6) ), num_faults), 2 );
        start_ind= start_ind + new_H;
        
    else
        
        new_H= nchoosek(obj.n_L_M, num_faults);
        obj.n_H= obj.n_H + new_H;
        obj.inds_H( start_ind:start_ind+new_H - 1, 1 )=...
            num2cell( nchoosek(1:obj.n_L_M, num_faults), 2 );
        start_ind= start_ind + new_H;
        
    end
end
obj.inds_H(start_ind:end)= [];



end
