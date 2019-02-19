
clear; clc;

%% decreasing separation on landmarks in y-coordinate
s_0= 30; % x-coordinate of the first pair of landmarks
s_step= 15; % x-coordinate separation
t_0= 20; % y-coordinate of the first pair of landmarks
t_step= 0.2; % y-coordinate step closer
t_threshold= 1; % y-coordinate minimum distance

% place first pair of landmarks
landmark_map= [s_0, t_0; s_0, -t_0];

s= s_0;
t= t_0;
while t > t_threshold
    
    s= s + s_step;
    t= t - t_step;
    
    landmarks= [s, t; s, -t];
    landmark_map= [landmark_map; landmarks];
end

%% decreasing separation on landmarks in x-coordinate

s_0= 30; % x-coordinate of the first pair of landmarks
s_step= 15; % x-coordinate separation
s_step_decrement= 0.8; % x-coordinate separation difference
t_0= 20; % y-coordinate of the first pair of landmarks
s_threshold= 1; % x-coordinate minimum distance

% place first pair of landmarks
landmark_map= [s_0, t_0; s_0, -t_0];

s= s_0;
t= t_0;
while s_step > s_threshold
    
    
    s= s + s_step;
    s_step= s_step - s_step_decrement;
        
    landmarks= [s, t_0; s, -t_0];
    landmark_map= [landmark_map; landmarks];
end

%% different sections with different separation

n_lm= 2; % number of landmarks for this section
s_0= 10; % x-coordinate of the first pair of landmarks
s_step= 15; % x-coordinate separation
t_0= 20; % y-coordinate of the first pair of landmarks

% place first pair of landmarks
% landmark_map= [s_0, t_0; s_0, -t_0];
landmark_map= [s_0, t_0];

% first section (well spaced)
s= s_0;
t= t_0;
for i= 1:n_lm
    
    s= s + s_step;
            
%     landmarks= [s, t_0; s, -t_0];
    landmarks= [s, t_0*(-1)^i];
    landmark_map= [landmark_map; landmarks];
end


n_lm= 1;
s_0= s + 45; % x-coordinate of the first pair of landmarks
s_step= 30; % x-coordinate separation
t_0= 20; % y-coordinate of the first pair of landmarks
t_step= 1;

% place first pair of landmarks
landmark_map= [landmark_map; s_0, t_0; s_0, -t_0];
landmark_map= [landmark_map; s_0, t_0+t_step; s_0, -t_0-t_step];


s= s_0;
for i= 1:n_lm
    
    s= s + s_step;
    
    landmark_map= [landmark_map; s, t_0; s, -t_0];
    landmark_map= [landmark_map; s, t_0 + t_step; s, -t_0 - t_step];
end





%%

landmark_map=...
    [20, 8;...
     20, 12;...
     25, 8;...
     25, 12;...
     -20, 8;...
     -20, 12;...
     -25, 8;...
     -25, 12;...
     ];
     
%%    
landmark_map=...
    [50, 5;...
     60, 5;...
     -50, 5;...
     -60, 5;...
    ];

%%

landmark_map=...
    [100, 0;...
     105, 0;...
     -100, 0;...
     -105, 0;...
     ];
    


