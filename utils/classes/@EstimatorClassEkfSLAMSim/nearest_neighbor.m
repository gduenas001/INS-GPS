
function nearest_neighbor(obj, z, params)

% number of features
obj.num_of_extracted_features= size(z,1);
obj.num_of_estimated_landmarks= (length(obj.XX) - params.m) / params.m_F;

% initialize with zero, if SLAM --> initialize with (-1)
%obj.association= zeros(obj.num_of_extracted_features, 1);
obj.association= ones(obj.num_of_extracted_features, 1) * (-1);

if obj.num_of_extracted_features == 0 || obj.num_of_estimated_landmarks == 0, return, end

% initialize variables
spsi= sin(obj.XX(3));
cpsi= cos(obj.XX(3));
zHat= zeros(params.m_F,1);

% % select landmarks in the field of view
% FoV_extension= max(5, 7*params.sig_lidar); % add this to the FoV
% obj.FoV_landmarks_at_k= zeros(obj.num_landmarks,1);
% for i= 1:obj.num_landmarks
%     
%     dx= obj.landmark_map(i,1) - obj.XX(1);
%     if abs(dx) > params.lidarRange + FoV_extension, continue, end
%     dy= obj.landmark_map(i,2) - obj.XX(2);
%     if abs(dy) > params.lidarRange +  + FoV_extension, continue, end
%     
%     % add 3 sigma to the extended field of view
%     if sqrt( dx^2 + dy^2 ) <= params.lidarRange + FoV_extension
%         obj.FoV_landmarks_at_k(i)= i;
%     end
% end
% % remove the ones that are zeros
% obj.FoV_landmarks_at_k( obj.FoV_landmarks_at_k == 0 )= [];

% Loop over extracted features
for i= 1:obj.num_of_extracted_features
    %min_y2= params.T_NN;
    min_y2= params.threshold_new_landmark;
    
    % loop through landmarks
    %for l= 1:length(obj.FoV_landmarks_at_k)
    for l= 1:obj.num_of_estimated_landmarks
        %lm_ind= obj.FoV_landmarks_at_k(l);
        lm_ind= (params.m + (params.m_F*l-1)):(params.m + params.m_F*l);
        %landmark= obj.landmark_map( lm_ind,: );
        
        dx= obj.XX(lm_ind(1)) - obj.XX(1);
        dy= obj.XX(lm_ind(2)) - obj.XX(2);
        
%         % TODO: I don't think this is needed, it has been checked before
%         dx= landmark(1) - obj.XX(1);
%         if abs(dx) > params.lidarRange, continue, end
%         dy= landmark(2) - obj.XX(2);
%         if abs(dy) > params.lidarRange, continue, end      
        
        % build innovation vector
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        % % quick check (10 m in X or Y)
        % if abs(gamma(1)) > 10 || abs(gamma(2)) > 10, continue, end
        
%         % Jacobian
%         H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
%              spsi, -cpsi, -dx*cpsi - dy*spsi ];
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
            spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];

        % covariance matrix
        % Y= H * obj.PX * H' + params.R_lidar;
        Y= H * obj.PX([1:params.m,lm_ind],[1:params.m,lm_ind]) * H' + params.R_lidar;
        
        % IIN squared
        y2= gamma' / Y * gamma;
        
        if y2 < min_y2
            min_y2= y2;
            obj.association(i)= l;
        end
        
    end
    
    % If the minimum value is very large --> new landmark
    if min_y2 > params.T_NN && min_y2 < params.threshold_new_landmark
        obj.association(i)= 0;
    end
    % Increase appearances counter
    % if obj.association(i) ~= 0  
    %     obj.appearances(obj.association(i))= obj.appearances(obj.association(i)) + 1;        
    % end
    if obj.association(i) ~= -1 && obj.association(i) ~= 0
        obj.appearances(obj.association(i))= obj.appearances(obj.association(i)) + 1;
    end
end
end

