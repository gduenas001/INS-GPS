
function [x,P]= GPS_update(x,P,z,R,minVelocityGPS,flagVel)

if norm(z(4:6)) > minVelocityGPS && flagVel % sense velocity
    R= diag( R );
    H= [eye(6), zeros(6,9)]; 
    disp('GPS velocity')
else
    z= z(1:3);
    R= diag( R(1:3) );
    H= [eye(3), zeros(3,12)]; 
    disp('-------- no GPS velocity ---------')
end

x(9)= pi_to_pi(x(9));
L= P*H' / (H*P*H' + R);
innov= z - H*x;

x= x + L*innov;
P= P - L*H*P;



