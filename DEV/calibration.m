
function [x,P]= calibration(x,P,z,H,R)

% Calibration msmt update
L= P*H' / (H*P*H' + R);
z_hat= H*x;
innov= z - z_hat;
innov(end)= pi_to_pi(innov(end));
x= x + L*innov;
P= P - L*H*P;






























