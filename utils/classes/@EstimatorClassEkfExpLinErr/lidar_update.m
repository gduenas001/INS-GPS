function lidar_update(obj, z, params)
option = optimoptions('fmincon','display','off');

obj.XX(params.ind_yaw)= pi_to_pi( obj.XX(params.ind_yaw) );

if all(obj.association == 0)
    obj.n_k= 0;
    obj.Y_k= [];
%     obj.Y_k_ub=[];
    obj.L_k= [];
%     obj.L_k_ub=[];
    obj.gamma_k= [];
    obj.q_k= 0;
    obj.H_k= [];
    obj.number_of_associated_LMs= 0;
    return;
end

% Eliminate the non-associated features
z(obj.association == 0, :)= [];

% number of associated features
obj.n_k= length(obj.association_no_zeros) * params.m_F;



%Build Jacobian H
R= kron( params.R_lidar, eye( obj.n_k / params.m_F ) );
obj.H_k= zeros(obj.n_k, length(obj.XX));
spsi= sin(obj.XX(params.ind_yaw));
cpsi= cos(obj.XX(params.ind_yaw));
zHat= zeros(obj.n_k,1);
Hu = zeros(2*obj.n_k / params.m_F);
for i= 1:length(obj.association_no_zeros)
    % Indexes
    indz= 2*i + (-1:0);

    dx= obj.landmark_map(obj.association_no_zeros(i), 1) - obj.XX(1);
    dy= obj.landmark_map(obj.association_no_zeros(i), 2) - obj.XX(2);

    % Predicted measurement
    zHat(indz)= [dx*cpsi + dy*spsi;
                -dx*spsi + dy*cpsi];

    % Jacobian -- H
    obj.H_k(indz,1)= [-cpsi; spsi];
    obj.H_k(indz,2)= [-spsi; -cpsi];
    obj.H_k(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];

%     X_min= [icdf('Normal',params.range_req/2,obj.XX(1),sqrt(obj.PX(1,1))),icdf('Normal',params.range_req/2,obj.XX(2),sqrt(obj.PX(2,2))),icdf('Normal',params.range_req/2,obj.XX(params.ind_yaw),sqrt(obj.PX(params.ind_yaw,params.ind_yaw)))];
%     X_max= [obj.XX(1)+ (obj.XX(1) - X_min(1)),obj.XX(2)+ (obj.XX(2) - X_min(2)),obj.XX(params.ind_yaw)+ (obj.XX(params.ind_yaw) - X_min(3))];
% 
%     hessx = @(x) -sum(abs(diag(sqrtm(obj.PX_ub)*[zeros(2,8), [sin(x(3)); -cos(x(3))], zeros(2,6); zeros(6,15); sin(x(3)), -cos(x(3)), zeros(1,6), -(obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*cos(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*sin(x(3)), zeros(1,6); zeros(6,15)]*sqrtm(obj.PX_ub))));
%     hessy = @(x) -sum(abs(diag(sqrtm(obj.PX_ub)*[zeros(2,8), [cos(x(3));  sin(x(3))], zeros(2,6); zeros(6,15); cos(x(3)),  sin(x(3)), zeros(1,6),  (obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*sin(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*cos(x(3)), zeros(1,6); zeros(6,15)]*sqrtm(obj.PX_ub))));
%     
%     [~,bvx] = fmincon(hessx,[obj.XX(1),obj.XX(2),obj.XX(params.ind_yaw)],[],[],[],[],X_min,X_max,[],option);
%     [~,bvy] = fmincon(hessy,[obj.XX(1),obj.XX(2),obj.XX(params.ind_yaw)],[],[],[],[],X_min,X_max,[],option);
%      Hu(indz,indz) = [bvx^2, 0;
%                      0, bvy^2];
end

% Update
% obj.Y_k_ub= obj.H_k * obj.PX_ub * obj.H_k' + (obj.Dc_cov_controller/4).*Hu + R;
obj.Y_k= obj.H_k * obj.PX * obj.H_k' + R;
% obj.L_k_ub= obj.PX_ub * obj.H_k' / obj.Y_k_ub;
obj.L_k= obj.PX * obj.H_k' / obj.Y_k;
zVector= z'; zVector= zVector(:);
obj.gamma_k= zVector - zHat;
obj.q_k= obj.gamma_k' / obj.Y_k * obj.gamma_k;
obj.XX= obj.XX + obj.L_k * obj.gamma_k;
% obj.PX_ub = obj.PX_ub - obj.L_k_ub * obj.H_k * obj.PX_ub;
obj.PX= obj.PX - obj.L_k * obj.H_k * obj.PX;

obj.number_of_associated_LMs= length(obj.association_no_zeros);
end
