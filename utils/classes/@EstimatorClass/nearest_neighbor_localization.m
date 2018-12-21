function association= nearest_neighbor_localization(obj, z, params)

n_F= size(z,1);
n_L= obj.num_landmarks;
obj.associated_landmarks_at_k = [];
obj.FoV_landmarks_at_k = [];
% initialize with zero, if SLAM --> initialize with (-1)
association= zeros(1,n_F);

if n_F == 0 || n_L == 0, return, end

spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));
zHat= zeros(2,1);
% Loop over extracted features
for i= 1:n_F
    minY= params.T_NN;
    
    for l= 1:n_L
        landmark= obj.landmark_map(l,:);
        
        dx= landmark(1) - obj.XX(1);
        dy= landmark(2) - obj.XX(2);
        
        if ( sqrt( dx^2 + dy^2 ) <= params.lidarRange ) && (i == 1)% build FoV landmarks in only one loop
            obj.FoV_landmarks_at_k = [ obj.FoV_landmarks_at_k; l];
        end
        
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
            spsi, -cpsi, -dx*cpsi - dy*spsi ];
        
        Y= H * obj.PX([1:2,9],[1:2,9]) * H' + params.R_lidar;
        
        y2= gamma' / Y * gamma;
        
        if y2 < minY
            minY= y2;
            association(i)= l;
        end
    end
    
    % If the minimum value is very large --> ignore
    if minY >= params.T_NN
        association(i)= 0;
    else % Increase appearances counter
        obj.appearances(association(i))= obj.appearances(association(i)) + 1;
        
        if sum(obj.associated_landmarks_at_k == association(i)) == 0
            obj.associated_landmarks_at_k=[obj.associated_landmarks_at_k;association(i)];
            
            % add associated landmark to FoV landmarks if it is not already
            % in there
            if sum(obj.FoV_landmarks_at_k == association(i)) == 0
                obj.FoV_landmarks_at_k = [ obj.FoV_landmarks_at_k; association(i)];
            end
        end
        
    end
end
end