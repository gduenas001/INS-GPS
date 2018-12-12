
function [idf,Nza]= data_associate_LNN_LS (z, R_lidar, T_NN, lidarRange)
% idf: row vector with the landmark association of each extracted feature
% in z
% Nza: number of associated features (the ones with a nonzero association)


global XX LM hlm Hlm PARAMS

Nz= size(z,1);
n_L= size(LM,1);

% Initialize associations
idf= zeros(1,Nz);

% Initialize lm models to be filled in the next function
hlm= cell(n_L,1);
Hlm= cell(n_L,1);

% Get all visible landmarks, assuming no mis-extractions here
% lm_ind= get_visible_landmarks(XX, lidarRange, 0);
lm_ind= 1:size(LM,1); 

% Create the nis table
IIN2_star= ones(1,Nz)*T_NN;
for i= 1:Nz
    for l= 1:n_L
        
        % It's in the extended FV and not removed by LS
%         if ismember(l,lm_ind)% && ~ismember(l,remove_lm_ind) 
            IIN2= lm_model_and_IIVN2(z(i,:)', R_lidar, l);
            if IIN2 < IIN2_star(i) % validated and the smallest IIN
                IIN2_star(i)= IIN2;
                idf(i)= l;
            end
%         end
        
    end
end

Nza= nnz(idf); 


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [IIN2]= lm_model_and_IIVN2(z,R,lm_id)
% return normalised innovation squared (ie, Mahalanobis distance)

global XX PX LM hlm Hlm

% auxiliary values
dx= LM(lm_id,1) - XX(1); 
dy= LM(lm_id,2) - XX(2);
d2= dx^2 + dy^2;
d= sqrt(d2);

xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
h= [d;
    atan2(dy,dx) - XX(9)];

% calculate H
H = [-xd -yd 0; 
      yd2 -xd2 -1];

% Store the values in the global varibles
hlm{lm_id}= h;
Hlm{lm_id}= H;


% Innovation vector
v= z-h; v(2)= pi_to_pi(v(2));

% % Quick check to discard associations
% if abs(v(1)) > 10 || abs(v(2)) > deg2rad(45), nis= inf; return, end;

% Compute IIVN 
S= H* PX([1:2,9], [1:2,9]) *H' + R; 

IIN2= v'/S*v;
































