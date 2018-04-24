clear; format short;% clc; close all;      


% ---------------- Read data X gravity ----------------
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF262/X_gravity/20180403_1.txt';
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/X_gravity/20180404_1.txt';
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/X_gravity/20180423_1.txt';
file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/X_gravity/20180423_2.txt';
[~, gyrox, gyroy, gyroz, accx, accy, accz,incx,incy,incz,...
    ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_x= length(accx);

% accelerometers
g_x= abs(mean(accx));
mux_y= mean(accy);
mux_z= mean(accz);

% gyros
wx_x= mean(gyrox);
wx_y= mean(gyroy);
wx_z= mean(gyroz);

% inclinometers
ig_x= abs(mean(incx));
imux_y= mean(incy);
imux_z= mean(incz);


% ---------------- Read data Y gravity ----------------
% file= strcat('../DATA_STATIC/3axis_calibration_125Hz_LPF262/Y_gravity/20180403_1.txt');
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Y_gravity/20180404_1.txt';
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Y_gravity/20180423_1.txt';
file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Y_gravity/20180423_2.txt';
[~, gyrox, gyroy, gyroz, accx, accy, accz,incx,incy,incz,...
    ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_y= length(accx);

% accelerometers
g_y= abs(-mean(accy));
muy_x= mean(accx);
muy_z= mean(accz);

% gyros
wy_x= mean(gyrox);
wy_y= mean(gyroy);
wy_z= mean(gyroz);

% inclinometers
ig_y= abs(-mean(incy));
imuy_x= mean(incx);
imuy_z= mean(incz);


% ---------------- Read data Z gravity ----------------
% file= strcat('../DATA_STATIC/3axis_calibration_125Hz_LPF262/Z_gravity/20180403_1.txt');
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Z_gravity/20180404_1.txt';
% file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Z_gravity/20180423_1.txt';
file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/Z_gravity/20180423_2.txt';
[~, gyrox, gyroy, gyroz, accx, accy, accz,incx,incy,incz,...
    ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_z= length(accx);

% accelerometers
g_z= abs(-mean(accz));
muz_x= mean(accx);
muz_y= mean(accy);

% gyros
wz_x= mean(gyrox);
wz_y= mean(gyroy);
wz_z= mean(gyroz);

% inclinometers
ig_z= abs(-mean(incz));
imuz_x= mean(incx);
imuz_y= mean(incy);


% ---------------- Final values ----------------
mu_x= ( muy_x * N_y + muz_x * N_z ) / ( N_y + N_z );
mu_y= ( mux_y * N_x + muz_y * N_z ) / ( N_x + N_z );
mu_z= ( mux_z * N_x + muy_z * N_y ) / ( N_x + N_y );
w_x= (wx_x * N_x + wy_x * N_y + wz_x * N_z ) / (N_x + N_y + N_z);
w_y= (wx_y * N_x + wy_y * N_y + wz_y * N_z ) / (N_x + N_y + N_z);
w_z= (wx_z * N_x + wy_z * N_z + wz_x * N_z ) / (N_x + N_y + N_z);
imu_x= ( imuy_x * N_y + imuz_x * N_z ) / ( N_y + N_z );
imu_y= ( imux_y * N_x + imuz_y * N_z ) / ( N_x + N_z );
imu_z= ( imux_z * N_x + imuy_z * N_y ) / ( N_x + N_y );


% ---------------- Calculate parameters ----------------
g= 9.80279; % Value of g at the IIT
% Accelerometers
c_x= (1/g) * (g_x - mu_x);
c_y= (1/g) * (g_y - mu_y);
c_z= (1/g) * (g_z - mu_z);
b_x0= mu_x / c_x;
b_y0= mu_y / c_y;
b_z0= mu_z / c_z;
% Inclinometers
ic_x= (1/g) * (ig_x - imu_x);
ic_y= (1/g) * (ig_y - imu_y);
ic_z= (1/g) * (ig_z - imu_z);
ib_x0= imu_x / ic_x;
ib_y0= imu_y / ic_y;
ib_z0= imu_z / ic_z;


% Form matrices
invC= diag( [c_x^(-1), c_y^(-1), c_z^(-1)] )
b_0= [b_x0; b_y0; b_z0; w_x; w_y; w_z]
iinvC= diag( [ic_x^(-1), ic_y^(-1), ic_z^(-1)] )
ib_0= [ib_x0; ib_y0; ib_z0]




