
function [phi,theta]= initial_attitude(iu)

g_bar= mean(iu,2);

% My method -- works for z-axis pointing down (accz < 0)
% theta=  atan2( g_bar(1) , abs(g_bar(3)) ); 
% phi=   -atan2( g_bar(2) , abs(g_bar(3)) );

% Books method
g_bar= -g_bar;
phi= atan2( g_bar(2),g_bar(3) );
theta= atan2( -g_bar(1), sqrt( g_bar(2)^2 + g_bar(3)^2 ) );



