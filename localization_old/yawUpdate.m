
function yawUpdate(w,R,r_IMU2rearAxis)

global XX PX


H= zeros(1, 15);
H(9)= 1;

z= yawMeasurement(w,r_IMU2rearAxis);
L= PX*H' / (H*PX*H' + R);
innov= z - H*XX;
innov= pi_to_pi(innov);
XX(9)= pi_to_pi(XX(9));
XX= XX + L*innov;
PX= PX - L*H*PX;




function yaw= yawMeasurement(w,r_IMU2rearAxis)

global XX

r= [-r_IMU2rearAxis;0;0];
v_o= XX(4:6);
R_NB= R_NB_rot(XX(7),XX(8),XX(9));

v_a= v_o + R_NB * cross(w,r);
v_a= v_a / norm(v_a);

yaw= atan2(v_a(2),v_a(1));








