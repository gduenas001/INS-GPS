function initialize_pitch_and_roll(obj, imu_calibration_msmts)
% calculates the initial pitch and roll

% compute gravity from static IMU measurements
g_bar= mean( imu_calibration_msmts, 2 );

% Books method
g_bar= -g_bar;
obj.XX(7)= atan2( g_bar(2),g_bar(3) );
obj.XX(8)= atan2( -g_bar(1), sqrt( g_bar(2)^2 + g_bar(3)^2 ) );

% My method -- works for z-axis pointing down (accz < 0)
% theta=  atan2( g_bar(1) , abs(g_bar(3)) );
% phi=   -atan2( g_bar(2) , abs(g_bar(3)) );
end