% ----------------------------------------------
% ----------------------------------------------
function association= nearest_neighbor_slam(obj, z, params)

n_F= size(z,1);
n_L= (length(obj.XX) - 15) / 2;

association= ones(1,n_F) * (-1);

if n_F == 0 || n_L == 0, return, end


spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));
zHat= zeros(2,1);
% Loop over extracted features
for i= 1:n_F
    minY= params.threshold_new_landmark;
    
    for l= 1:n_L
        ind= (15 + (2*l-1)):(15 + 2*l);
        
        dx= obj.XX(ind(1)) - obj.XX(1);
        dy= obj.XX(ind(2)) - obj.XX(2);
        
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
            spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];
        
        Y= H * obj.PX([1:2,9,ind],[1:2,9,ind]) * H' + params.R_lidar;
        
        y2= gamma' / Y * gamma;
        
        if y2 < minY
            minY= y2;
            association(i)= l;
        end
    end
    
    % If the minimum value is very large --> new landmark
    if minY > params.T_NN && minY < params.threshold_new_landmark
        association(i)= 0;
    end
end

% Increase appearances counter
for i= 1:n_F
    if association(i) ~= -1 && association(i) ~= 0
        obj.appearances(association(i))= obj.appearances(association(i)) + 1;
    end
end
end