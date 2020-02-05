function lidar_update(obj, z, params)
option = optimoptions('fmincon','display','off');
obj.XX(params.ind_yaw)= pi_to_pi( obj.XX(params.ind_yaw) );


if all(obj.association == 0)
    obj.n_k= 0;
    obj.Y_k= [];
    obj.Y_k_ub=[];
    obj.L_k= [];
    obj.L_k_ub= [];
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

    % Hessian
    % range for bounding
      %x_range = -10:0.1:10;
      %X_range = [x_range' x_range' x_range'];
      %p = mvncdf(X_range,obj.XX',obj.PX);
      %cdf_id = find(p>1e-05 & p<1-1e-05);
      %x_range = x_range(cdf_id);
      %X_range = obj.PX*[x_range; x_range; x_range]+obj.XX;

      X_min= [icdf('Normal',params.range_req/2,obj.XX(1),sqrt(obj.PX_ub(1,1))),icdf('Normal',params.range_req/2,obj.XX(2),sqrt(obj.PX_ub(2,2))),icdf('Normal',params.range_req/2,obj.XX(3),sqrt(obj.PX_ub(3,3)))];
      X_max= [obj.XX(1)+ (obj.XX(1) - X_min(1)),obj.XX(2)+ (obj.XX(2) - X_min(2)),obj.XX(3)+ (obj.XX(3) - X_min(3))];

% %     % bound for linearization error
% %     % tight bound
% %     %hessx = @(x) -(2*obj.PX(1,3)*sin(x(3))-2*obj.PX(2,3)*cos(x(3))-obj.PX(3,3)*((obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*cos(x(3))+(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*sin(x(3))));
% %     %hessy = @(x) -(2*obj.PX(1,3)*cos(x(3))+2*obj.PX(2,3)*sin(x(3))+obj.PX(3,3)*((obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*sin(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*cos(x(3))));
% %     % loose bound
% %     hessx = @(x) -(abs(obj.PX(1,3))+abs(obj.PX(2,3))+abs(obj.PX(1,3)*sin(x(3))-obj.PX(2,3)*cos(x(3))-obj.PX(3,3)*((obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*cos(x(3))+(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*sin(x(3)))));
% %     hessy = @(x) -(abs(obj.PX(1,3))+abs(obj.PX(2,3))+abs(obj.PX(1,3)*cos(x(3))+obj.PX(2,3)*sin(x(3))+obj.PX(3,3)*((obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*sin(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*cos(x(3)))));
      hessx = @(x) -sum(abs(diag(sqrtm(obj.PX_ub)*[0 0 sin(x(3)); 0 0 -cos(x(3)); sin(x(3)) -cos(x(3)) -(obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*cos(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*sin(x(3))]*sqrtm(obj.PX_ub))));
      hessy = @(x) -sum(abs(diag(sqrtm(obj.PX_ub)*[0 0 cos(x(3)); 0 0  sin(x(3)); cos(x(3))  sin(x(3))  (obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*sin(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*cos(x(3))]*sqrtm(obj.PX_ub))));

      %[~,bvx] = fmincon(hessx,obj.XX,[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
      [~,bvx] = fmincon(hessx,obj.XX,[],[],[],[],X_min,X_max,[],option);
      %[~,bvy] = fmincon(hessy,obj.XX,[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
      [~,bvy] = fmincon(hessy,obj.XX,[],[],[],[],X_min,X_max,[],option);
% % %     [~,bvx] = fmincon(hessx,[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
% % %     [~,bvy] = fmincon(hessy,[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
       Hu(indz,indz) = [bvx^2, 0;
                       0, bvy^2];
end

% Update

obj.Y_k_ub= obj.H_k * obj.PX_ub * obj.H_k' + (obj.Dc_cov_controller/4).*Hu + R;
obj.Y_k= obj.H_k * obj.PX * obj.H_k' + R;
obj.L_k_ub= obj.PX_ub * obj.H_k' / obj.Y_k_ub;
obj.L_k= obj.PX * obj.H_k' / obj.Y_k;
zVector= z'; zVector= zVector(:);
obj.gamma_k= zVector - zHat;
obj.q_k= obj.gamma_k' / obj.Y_k * obj.gamma_k;
obj.XX= obj.XX + obj.L_k * obj.gamma_k;
obj.PX_ub= obj.PX_ub - obj.L_k_ub * obj.H_k * obj.PX_ub;
obj.PX= obj.PX - obj.L_k * obj.H_k * obj.PX;
obj.number_of_associated_LMs= length(obj.association_no_zeros);
end
