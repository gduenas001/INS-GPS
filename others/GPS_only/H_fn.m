function H = H_fn(vx,vy,vz,phi,theta,psi)
%H_FN
%   H = H_fn(vx,vy,vz,phi,theta,psi)

t2 = cos(psi);
t3 = sin(phi);
t4 = cos(phi);
t5 = sin(psi);
t6 = sin(theta);
t7 = cos(theta);
t8 = t4.*t5.*t6;
t9 = t8-t2.*t3;
t10 = t3.*t5;
t11 = t2.*t4.*t6;
t12 = t10+t11;
H = [0.0,0.0,0.0,t12,t9,t4.*t7,vx.*(t4.*t5-t2.*t3.*t6)-vy.*(t2.*t4+t3.*t5.*t6)-t3.*t7.*vz,t4.*(-t6.*vz+t2.*t7.*vx+t5.*t7.*vy),-t9.*vx+t12.*vy,0.0,0.0,0.0,0.0,0.0,0.0];
