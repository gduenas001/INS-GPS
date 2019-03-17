function compute_B_bar_matrix(obj, estimator)

% Augmented B
obj.B_bar= inf* ones( obj.n_M , obj.n_M + obj.m );
A_prev= obj.Lpp_k \ obj.A_M( : , estimator.n_k + 1:end );
B_ind_row_start= estimator.n_k + 1;
B_ind_col_end= estimator.n_k;

% accounting for the case where there are no landmarks in the FoV at epoch k
if estimator.n_k > 0
    obj.B_bar(1:estimator.n_k , :)=...
    [ eye(estimator.n_k), -obj.H_k * obj.Phi_ph{1} * A_prev ];
end

% Recursive computation of B
for i= 1:obj.M
    A_prev= obj.Lpp_ph{i} \ A_prev(:, obj.n_ph(i)+1:end);
    
    % accounting for the case where there are no landmarks in the FoV at
    % one of the epochs in the preceding horizon
    if obj.n_ph(i) > 0
        B= [eye( obj.n_ph(i) ) , -obj.H_ph{i} * obj.Phi_ph{i+1} * A_prev];

        B_ind_row_end= B_ind_row_start + obj.n_ph(i) - 1;

        obj.B_bar(B_ind_row_start:B_ind_row_end, 1:B_ind_col_end)= 0;
        obj.B_bar(B_ind_row_start:B_ind_row_end, B_ind_col_end+1:end)= B;

        % increase row index for next element B
        B_ind_row_start= B_ind_row_start + obj.n_ph(i);
        B_ind_col_end= B_ind_col_end + obj.n_ph(i);
    end
    
end
end
