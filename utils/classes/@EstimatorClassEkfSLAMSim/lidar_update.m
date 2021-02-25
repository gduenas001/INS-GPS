function lidar_update(obj, z, params)

obj.XX(params.ind_yaw)= pi_to_pi( obj.XX(params.ind_yaw) ); 


%if all(obj.association == 0)
%    obj.n_k= 0;
%    obj.Y_k= [];
%    obj.L_k= [];
%    obj.gamma_k= [];
%    obj.q_k= 0;
%    obj.H_k= [];
%    obj.number_of_associated_LMs= 0;
%    return;
%end

association = obj.association;

if all(association == -1), return; end

% Eliminate the non-associated features
ind_to_eliminate= association == -1 | association == 0;
z(ind_to_eliminate,:)= [];
association(ind_to_eliminate) = [];

% % Eliminate the non-associated features
% z(obj.association == 0, :)= [];

if isempty(z)
    obj.n_k= 0;
    obj.Y_k= [];
    obj.L_k= [];
    obj.gamma_k= [];
    obj.q_k= 0;
    obj.H_k= [];
    obj.number_of_associated_LMs= 0;
    return;
end

% number of associated features
%obj.n_k= length(obj.association_no_zeros) * params.m_F;
obj.n_k= length(association) * params.m_F;


%Build Jacobian H
R= kron( params.R_lidar, eye( obj.n_k / params.m_F ) );
obj.H_k= zeros(obj.n_k, length(obj.XX));
spsi= sin(obj.XX(params.ind_yaw));
cpsi= cos(obj.XX(params.ind_yaw));
zHat= zeros(obj.n_k,1);
%for i= 1:length(obj.association_no_zeros)
for i= 1:length(association)
    % Indexes
    indz= params.m_F*i + (-1:0);
    indx= params.m + params.m_F*association(i) + (-1:0);
    
    dx= obj.XX(indx(1)) - obj.XX(1);
    dy= obj.XX(indx(2)) - obj.XX(2);
    %dx= obj.landmark_map(obj.association_no_zeros(i), 1) - obj.XX(1);
    %dy= obj.landmark_map(obj.association_no_zeros(i), 2) - obj.XX(2);
    
    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
                -dx*spsi + dy*cpsi];
    
    % Jacobian -- H
    obj.H_k(indz,1)= [-cpsi; spsi];
    obj.H_k(indz,2)= [-spsi; -cpsi];
    obj.H_k(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
    obj.H_k(indz,indx)= [cpsi, spsi;
        -spsi, cpsi];
end

% Update
obj.Y_k= obj.H_k * obj.PX * obj.H_k' + R;
obj.L_k= obj.PX * obj.H_k' / obj.Y_k;
zVector= z'; zVector= zVector(:);
obj.gamma_k= zVector - zHat;
obj.q_k= obj.gamma_k' / obj.Y_k * obj.gamma_k;
obj.XX= obj.XX + obj.L_k * obj.gamma_k;
obj.PX= obj.PX - obj.L_k * obj.H_k * obj.PX;
obj.number_of_associated_LMs= length(association);
end