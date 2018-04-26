
function [x,P]= yawUpdate(x,P,w,H,R,r_IMU2rearAxis)

z= yawMeasurement(x,w,r_IMU2rearAxis);
L= P*H' / (H*P*H' + R);
innov= z - H*x;
innov= pi_to_pi(innov);
x(9)= pi_to_pi(x(9));
x= x + L*innov;
P= P - L*H*P;




function yaw= yawMeasurement(x,w,r_IMU2rearAxis)

r= [-r_IMU2rearAxis;0;0];
v_o= x(4:6);
R_NB= R_NB_rot(x(7),x(8),x(9));

v_a= v_o + R_NB * cross(w,r);
v_a= v_a / norm(v_a);

yaw= atan2(v_a(2),v_a(1));








