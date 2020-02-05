
function nearest_neighbor(obj, z, params)
%option = optimoptions('fmincon','Display','off');

% number of features
obj.num_of_extracted_features= size(z,1);

% initialize with zero, if SLAM --> initialize with (-1)
obj.association= zeros(obj.num_of_extracted_features, 1);

if obj.num_of_extracted_features == 0, return, end

% initialize variables
spsi= sin(obj.XX(3));
cpsi= cos(obj.XX(3));
zHat= zeros(2,1);

% select landmarks in the field of view
FoV_extension= max(5, 7*params.sig_lidar); % add this to the FoV
obj.FoV_landmarks_at_k= zeros(obj.num_landmarks,1);
for i= 1:obj.num_landmarks

    dx= obj.landmark_map(i,1) - obj.XX(1);
    if abs(dx) > params.lidarRange + FoV_extension, continue, end
    dy= obj.landmark_map(i,2) - obj.XX(2);
    if abs(dy) > params.lidarRange +  + FoV_extension, continue, end

    % add 3 sigma to the extended field of view
    if sqrt( dx^2 + dy^2 ) <= params.lidarRange + FoV_extension
        obj.FoV_landmarks_at_k(i)= i;
    end
end
% remove the ones that are zeros
obj.FoV_landmarks_at_k( obj.FoV_landmarks_at_k == 0 )= [];

% Loop over extracted features
for i= 1:obj.num_of_extracted_features
    min_y2= params.T_NN;

    % loop through landmarks
    for l= 1:length(obj.FoV_landmarks_at_k)
        lm_ind= obj.FoV_landmarks_at_k(l);
        landmark= obj.landmark_map( lm_ind,: );

        % TODO: I don't think this is needed, it has been checked before
        dx= landmark(1) - obj.XX(1);
        if abs(dx) > params.lidarRange, continue, end
        dy= landmark(2) - obj.XX(2);
        if abs(dy) > params.lidarRange, continue, end

        % build innovation vector
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;

        % quick check (10 m in X or Y)
        if abs(gamma(1)) > 10 || abs(gamma(2)) > 10, continue, end

        % Jacobian
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
             spsi, -cpsi, -dx*cpsi - dy*spsi ];

        % range for bounding
          %x_range = -10:0.1:10;
          %X_range = [x_range' x_range' x_range'];
          %obj.PX=(obj.PX+(obj.PX'))/2;
          %p = mvncdf(X_range,obj.XX',obj.PX);
          %cdf_id = find(p>1e-07 & p<1-1e-07);
          %x_range = x_range(cdf_id);
          %X_range = obj.PX*[x_range; x_range; x_range]+obj.XX;

% %         % bound for linearization error
% %         % strict bound
% %         %hessx = @(x) -(2*obj.PX(1,3)*sin(x(3))-2*obj.PX(2,3)*cos(x(3))-obj.PX(3,3)*((landmark(1)-x(1))*cos(x(3))+(landmark(2)-x(2))*sin(x(3))));
% %         %hessy = @(x) -(2*obj.PX(1,3)*cos(x(3))+2*obj.PX(2,3)*sin(x(3))+obj.PX(3,3)*((landmark(1)-x(1))*sin(x(3))-(landmark(2)-x(2))*cos(x(3))));
% %         % loose bound
% %          hessx = @(x) -(abs(obj.PX(1,3))+abs(obj.PX(2,3))+abs(obj.PX(1,3)*sin(x(3))-obj.PX(2,3)*cos(x(3))-obj.PX(3,3)*((landmark(1)-x(1))*cos(x(3))+(landmark(2)-x(2))*sin(x(3)))));
% %          hessy = @(x) -(abs(obj.PX(1,3))+abs(obj.PX(2,3))+abs(obj.PX(1,3)*cos(x(3))+obj.PX(2,3)*sin(x(3))+obj.PX(3,3)*((landmark(1)-x(1))*sin(x(3))-(landmark(2)-x(2))*cos(x(3)))));
           %hessx = @(x) -sum(abs(diag(sqrtm(obj.PX)*[0 0 sin(x(3)); 0 0 -cos(x(3)); sin(x(3)) -cos(x(3)) -(landmark(1)-x(1))*cos(x(3))-(landmark(2)-x(2))*sin(x(3))]*sqrtm(obj.PX))));
           %hessy = @(x) -sum(abs(diag(sqrtm(obj.PX)*[0 0 cos(x(3)); 0 0  sin(x(3)); cos(x(3))  sin(x(3))  (landmark(1)-x(1))*sin(x(3))-(landmark(2)-x(2))*cos(x(3))]*sqrtm(obj.PX))));
 
          %[~,bvx] = fmincon(hessx,obj.XX,[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
          %[~,bvy] = fmincon(hessy,obj.XX,[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
% %  %         [~,bvx] = fmincon(hessx,[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
% %  %         [~,bvy] = fmincon(hessy,[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],[],[],[],[min(X_range(1,:)),min(X_range(2,:)),min(X_range(3,:))],[max(X_range(1,:)),max(X_range(2,:)),max(X_range(3,:))],[],option);
            %Hu = [bvx^2, 0;
            %     0, bvy^2];
        % covariance matrix
       % Y= H * obj.PX * H' + (obj.Dc_cov_controller/4).*Hu + params.R_lidar;
        Y= H * obj.PX * H' + params.R_lidar;

        % IIN squared
        y2= gamma' / Y * gamma;

        if y2 < min_y2
            min_y2= y2;
            obj.association(i)= lm_ind;
        end
    end

    % Increase appearances counter
    if obj.association(i) ~= 0
        obj.appearances(obj.association(i))= obj.appearances(obj.association(i)) + 1;
    end
end
end
