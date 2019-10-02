
function compute_B_matrix_fg(obj,params, i, m_F)

% initialize fault type indicator vector (GPS or LiDAR)
fault_type_indicator= -1 * ones(length(i),1);

for j = 1 : length( i )
    if i(j) > obj.n_L_M
        % in the case of GPS
        fault_type_indicator(j)= 6;
    else
        % in the case of LiDAR
        fault_type_indicator(j)= m_F;
    end
end

%initialize faulted msmts indices vector
ind_faulted_msmts= zeros( sum(fault_type_indicator) , 1 );


% fetch the faulted msmts indices
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


% build the fault-free msmts extraction matrix
obj.B_j = zeros( obj.n_total-obj.m-sum(fault_type_indicator) , obj.n_total );
obj.B_j(1,3)=1;
obj.B_j(2,4)=1;
obj.B_j(3,5)=1;
obj.B_j(4,6)=1;
obj.B_j(5,7)=1;
obj.B_j(6,8)=1;

%Osama
%obj.B_j(7,10)=1;
%obj.B_j(8,11)=1;
%obj.B_j(9,12)=1;
%obj.B_j(10,13)=1;
%obj.B_j(11,14)=1;
%obj.B_j(12,15)=1;

tmp=7;

for j = (9+1):obj.n_total  %Osama params.m

    if sum(ind_faulted_msmts==j) == 1

        continue;

    else

        obj.B_j(tmp,j) = 1;

        tmp=tmp+1;

    end

end
    
end
