
function compute_B_matrix_fg(obj, i, m_F)

ind_faulted_msmts= zeros( length(i)*m_F , 1 );

for j = 1 : length( i )
    
    ind_faulted_msmts( m_F*(j-1)+1: m_F*j ) = obj.abs_msmt_ind(:,i(j));
    
end

obj.B_j = zeros( obj.n_total-obj.m-(m_F*length(i)) , obj.n_total );

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
