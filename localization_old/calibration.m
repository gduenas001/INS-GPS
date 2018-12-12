
function calibration(z,H,R)

global XX PX

% Calibration msmt update
L= PX(1:15,1:15)*H' / (H*PX(1:15,1:15)*H' + R);
z_hat= H*XX(1:15);
innov= z - z_hat;
innov(end)= pi_to_pi(innov(end));
XX(1:15)= XX(1:15) + L*innov;
PX(1:15,1:15)= PX(1:15,1:15) - L*H*PX(1:15,1:15);






























