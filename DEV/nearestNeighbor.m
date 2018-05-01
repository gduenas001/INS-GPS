
function [association,appearances]= nearestNeighbor(z,appearances,R,T,T_newLM)

global XX PX

n_F= size(z,1);
n_L= (length(XX) - 15) / 2;

association= ones(1,n_F) * (-1);

if n_F == 0 || n_L == 0, return, end


spsi= sin(XX(9));
cpsi= cos(XX(9));
zHat= zeros(2,1);
% Loop over extracted features
for i= 1:n_F
    minY= T_newLM;
    
    for l= 1:n_L
        ind= (15 + (2*l-1)):(15 + 2*l);
        
        dx= XX(ind(1)) - XX(1);
        dy= XX(ind(2)) - XX(2);
        
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
             spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];
         
        Y= H * PX([1:2,9,ind],[1:2,9,ind]) * H' + R;
        
        y2= gamma' / Y * gamma;
        
        if y2 < minY
            minY= y2;
            association(i)= l;
        end
    end
    
    % If the minimum value is very large --> new landmark
    if minY > T && minY < T_newLM
        association(i)= 0;
    end
end


% Increase appearances counter
for i= 1:n_F
    if association(i) ~= -1 && association(i) ~= 0
        appearances(association(i))= appearances(association(i)) + 1;
    end
end


end











