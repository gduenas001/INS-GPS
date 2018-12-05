
function [phi,theta]= initial_attitude(iu)

mu= mean(iu,2);

theta=  atan2( mu(1) , abs(mu(3)) ); % for z-axis pointing down (accz < 0)
phi=   -atan2( mu(2) , abs(mu(3)) );








