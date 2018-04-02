function [Fn, F, Gu_tilda, Gw, H]= Matrices(u10, u20, u30, wx0, wy0, wz0, tau)
%MATRICES Summary of this function goes here
% Detailed explanation goes here

% Define system Matrices Discrete KF
Fn= zeros(9);
Fn(1:3,4:6)= eye(3);

Fn(4:6,7:9)= (-1)* [0,  -u30, u20; 
                    u30, 0,  -u10;
                   -u20, u10, 0  ];
               
% K* or F_E -- Evaluated at gyros measured biased and identity R_BN
Fn(7:9,7:9)= [0,   wz0, 0; 
             -wz0, 0,   0;
              wy0, 0,   0];

F2= zeros(9,6);
F2(4:9,1:6)= eye(6);
F3= zeros(6,9);
F4= - eye(6) / tau;

% Aa 15 x 15
F= [Fn, F2;
    F3, F4];

% Control
Gu= zeros(9,6);
Gu(4:9,1:6)= eye(6);
Gu_tilda= [Gu; zeros(6)];

% Noise
Gw= [-Gu, zeros(9,6);
      zeros(6,6), eye(6)];

% Measurement matrix
H= zeros(3,15);
H(1:3,1:3)= eye(3);


end

