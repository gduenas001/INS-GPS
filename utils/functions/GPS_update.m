
function GPS_update(z,R,minVelocityGPS,flagVel)

global XX PX

n_L= (length(XX) - 15) / 2;

if norm(z(4:6)) > minVelocityGPS && flagVel % sense velocity
    R= diag( R );
    H= [eye(6), zeros(6,9), zeros(6,n_L*2)]; 
    disp('GPS velocity')
else
    z= z(1:3);
    R= diag( R(1:3) );
    H= [eye(3), zeros(3,12), zeros(3,n_L*2)]; 
    disp('-------- no GPS velocity ---------')
end

XX(9)= pi_to_pi(XX(9));
L= PX*H' / (H*PX*H' + R);
innov= z - H*XX;

XX= XX + L*innov;
PX= PX - L*H*PX;



