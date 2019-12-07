function SM_update(obj, z, R, params)


n_L= (length(obj.XX) - 15) / 2;

% update only the position, no velocity
z= z(1:6);
R= diag( R(1:6) );
H= [eye(3), zeros(3,12), zeros(3,n_L*2);
    zeros(3,6), eye(3),zeros(3,6), zeros(3,n_L*2)];
disp('-------- Scan matching contains no velocity info ---------')

obj.XX(9)= pi_to_pi( obj.XX(9) );
L= obj.PX*H' / (H*obj.PX*H' + R);
innov= z - H*obj.XX;

obj.XX= obj.XX + L*innov;
obj.PX= obj.PX - L*H*obj.PX;
end