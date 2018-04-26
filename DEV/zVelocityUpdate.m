
function [x,P]= zVelocityUpdate(x,P,R)


% Normalize yaw
x(9)= pi_to_pi( x(9) );

% Update
R_BN= R_NB_rot( x(7), x(8), x(9) )';
H= [0,0,1]*R_BN*[zeros(3),eye(3),zeros(3,9)];
L= P*H' / (H*P*H' + R);
z_hat= H * x;
innov= 0 - z_hat;
x= x + L*innov;
P= P - L*H*P;



% This is a different option to do the update in Z, but it is more
% computationally expensive and does not offer better results in my case
%{
R_BN= R_NB_rot( x(7,k+1), x(8,k+1), x(9,k+1) )';
H_virt= H_fn(x(4,k+1), x(5,k+1), x(6,k+1), x(7,k+1), x(8,k+1), x(9,k+1));
L= P*H_virt' / (H_virt*P*H_virt' + R_virt_Z);
z= 0;
z_hat= [0,0,1] * R_BN * x(4:6,k+1);
innov= z - z_hat;
x(:,k+1)= x(:,k+1) + L*innov;
P= P - L*H_virt*P;
%}