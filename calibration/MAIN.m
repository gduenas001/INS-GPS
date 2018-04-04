clear; format short; clc; close all;      


% ---------------- Read data X gravity ----------------
file= '../DATA_STATIC/3axis_calibration_125Hz_LPF16/X_gravity/20180404_1.txt';
[~, gyrox, gyroy, gyroz, accx, accy, accz,...
    ~, ~, ~, ~, ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_x= length(accx);

g_x= mean(accx);
mux_y= mean(accy);
mux_z= mean(accz);
wx_x= mean(gyrox);
wx_y= mean(gyroy);
wx_z= mean(gyroz);


% ---------------- Read data Y gravity ----------------
file= strcat('../DATA_STATIC/3axis_calibration_125Hz_LPF16/Y_gravity/20180404_1.txt');
[~, gyrox, gyroy, gyroz, accx, accy, accz,...
    ~, ~, ~, ~, ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_y= length(accx);

g_y= -mean(accy);
muy_x= mean(accx);
muy_z= mean(accz);
wy_x= mean(gyrox);
wy_y= mean(gyroy);
wy_z= mean(gyroz);

%{
% ---------------- Read data Z gravity ----------------
file= strcat('../DATA_STATIC/3axis_calibration_125Hz_LPF16/Z_gravity/20180404_1.txt');
[~, gyrox, gyroy, gyroz, accx, accy, accz,...
    ~, ~, ~, ~, ~, ~, ~, ~, ~]= DataRead(file); % rads
% -------------------------------------------

% Set paramters
u= [accx, accy, accz, gyrox, gyroy, gyroz]';
N_z= length(accx);

g_z= -mean(accz);
muz_x= mean(accx);
muz_y= mean(accy);
wz_x= mean(gyrox);
wz_y= mean(gyroy);
wz_z= mean(gyroz);


% ---------------- Final values ----------------
mu_x= ( muy_x * N_y + muz_x * N_z ) / ( N_y + N_z );
mu_y= ( mux_y * N_x + muz_y * N_z ) / ( N_x + N_z );
mu_z= ( mux_z * N_x + muy_z * N_y ) / ( N_x + N_y );
w_x= (wx_x * N_x + wy_x * N_y + wz_x * N_z ) / (N_x + N_y + N_z);
w_y= (wx_y * N_x + wy_y * N_y + wz_y * N_z ) / (N_x + N_y + N_z);
w_z= (wx_z * N_x + wy_z * N_z + wz_x * N_z ) / (N_x + N_y + N_z);


% ---------------- Calculate parameters ----------------
g= 9.80279; % Value of g at the IIT
c_x= (1/g) * (g_x - mu_x);
c_y= (1/g) * (g_y - mu_y);
c_z= (1/g) * (g_z - mu_z);
b_x0= mu_x / c_x;
b_y0= mu_y / c_y;
b_z0= mu_z / c_z;

% Form matrices
invC= diag( [c_x^(-1), c_y^(-1), c_z^(-1)] )
b_0= [b_x0; b_y0; b_z0; w_x; w_y; w_z]


%}


