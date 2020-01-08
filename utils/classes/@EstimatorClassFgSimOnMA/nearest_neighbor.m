
function nearest_neighbor(obj, z, params)

% number of features
num_of_extracted_features= size(z,2);

% initialize with zero, if SLAM --> initialize with (-1)
obj.association= zeros(num_of_extracted_features, 1);

% initialize variables
spsi= sin(obj.XX(3));
cpsi= cos(obj.XX(3));
z_estimated= zeros(2,1);

% select landmarks in the field of view
FoV_extension= max(5, 7*params.sig_lidar); % add this to the FoV

% Loop over extracted features
for i= 1:num_of_extracted_features
    min_y2= inf;%sqrt(params.T_NN);
    
    % loop through landmarks
    for l= 1:obj.num_landmarks
        
        landmark= obj.landmark_map( l,: );
        
        % TODO: I don't think this is needed, it has been checked before
        dx= landmark(1) - obj.XX(1);
        if abs(dx) > params.lidarRange + FoV_extension, continue, end
        dy= landmark(2) - obj.XX(2);
        if abs(dy) > params.lidarRange + FoV_extension, continue, end      
        
        % build innovation vector
        z_estimated(1)=  dx*cpsi + dy*spsi;
        z_estimated(2)= -dx*spsi + dy*cpsi;
        b_i_t = sqrtm(params.R_lidar)\(z(:,i) - z_estimated);
        
        % quick check (10 m in X or Y)
        if abs(gamma(1)) > 10 || abs(gamma(2)) > 10, continue, end
        
        H_t= zeros( params.m_F , (obj.M+1)*params.m );
        H_t(:,end-params.m+1)= [-cpsi;spsi];
        H_t(:,end-params.m+2)= [-spsi;-cpsi];
        H_t(:,end-params.m+params.ind_yaw)= [-dx*spsi + dy*cpsi;-dx*cpsi - dy*spsi ];
        
        A_t= params.R_lidar\H_t;
        n_total= size(obj.A,1);
        E_i= [zeros(params.m_F,n_total-(num_of_extracted_features*params.m_F)),zeros(params.m_F,(i-1)*params.m_F),eye(params.m_F),zeros(params.m_F,(num_of_extracted_features*params.m_F)-(i*params.m_F))];
        PX_M= inv(obj.A'*obj.A);
        R_i_t= (E_i-A_t*PX_M*obj.A')*(E_i-A_t*PX_M*obj.A')';
        
        % IRN
        y2 = sqrt( (b_i_t' /R_i_t) * b_i_t);
        
        if y2 < min_y2
            min_y2= y2;
            obj.association(i)= l;
        end
    end
    
end
end

