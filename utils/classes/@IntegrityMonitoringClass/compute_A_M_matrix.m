function compute_A_M_matrix(obj)
% create matrix A_M at k
obj.A_M= obj.L_k;
for i= 1:obj.M
    if i == 1
        Dummy_Variable= obj.Lpp_k;
    else
        Dummy_Variable= Dummy_Variable * obj.Lpp_ph{i-1};
    end
    obj.A_M= [obj.A_M , Dummy_Variable * obj.L_ph{i}];
end
obj.A_M= [ obj.A_M , Dummy_Variable * obj.Lpp_ph{obj.M} ];
end