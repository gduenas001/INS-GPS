
function compute_B_matrix_fg(obj, i, m_F)

fault_type_indicator= -1 * ones(length(i),1);

for j = 1 : length( i )
    if i(j) > obj.n_L_M
        fault_type_indicator(j)= 6;
    else
        fault_type_indicator(j)= m_F;
    end
end

ind_faulted_msmts= zeros( sum(fault_type_indicator) , 1 );

tmp=0;

for j = 1 : length( i )
    
    if fault_type_indicator(j) == m_F
        ind_faulted_msmts( tmp+1: tmp+m_F ) = obj.lidar_msmt_ind(:,i(j));
        tmp = tmp + m_F;
    else
        ind_faulted_msmts( tmp+1: tmp+6 ) = obj.gps_msmt_ind(:,i(j)-obj.n_L_M);
        tmp = tmp + 6;
    end
end

obj.B_j = zeros( obj.n_total-obj.m-sum(fault_type_indicator) , obj.n_total );

tmp=1;

for j = (obj.m+1):obj.n_total

    if sum(ind_faulted_msmts==j) == 1

        continue;

    else

        obj.B_j(tmp,j) = 1;

        tmp=tmp+1;

    end

end
    
end
