The organization of the GPS data is as follows:
Year Month Day Number_of_Seconds_from_the_begining_of_the_test_day:
Px Py Pz Vx Vy Vz sigPx sigPy sigPz sigVx sigVy sigVz

where:-
Px,Py,Pz: the positions(m) in cartesian coordinates using ECEF frame.
Vx,Vy,Vz: the velocities(m/s) in cartesian coordinates using ECEF frame.
sigPx sigPy sigPz: the positions' standard deviations(m) in cartesian coordinates using ECEF frame.
sigVx sigVy sigVz: the velocities' standard deviations(m/s) in cartesian coordinates using ECEF frame.

The organization of the IMU data is as follows:
Year Month Day Number_of_Seconds_from_the_begining_of_the_test_day GYRO_X GYRO_Y GYRO_Z GyroSts ACC_X ACC_Y ACC_Z AccSts INC_X INC_Y INC_Z IncSts

where:-
GYRO_X,GYRO_Y,GYRO_Z: Gyroscopes' readings(deg/s) in cartestian coordinates using Body frame.
ACC_X,ACC_Y,ACC_Z: Accelerometers' readings(g) in cartestian coordinates using Body frame.
INC_X INC_Y INC_Z: Inclinometers' readings(g) in cartestian coordinates using Body frame.
GyroSts,AccSts,IncSts: Status of sensors' readings (0:Correct,1:Incorrect)

__________________________________________________________________________________________________________________________________________________

It is important to note that the axes of the IMU is aligned with the axes of the GPS. However, there is an offset in the origin of the GPS relative to the origin of the IMU. This offset must be added to the readings of the GPS, which is expressed as follows:
Offset_X= 0 m
Offset_Y= 0 m
Offset_Z= 2.00 m