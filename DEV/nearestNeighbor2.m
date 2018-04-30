
function [association,appearances]= nearestNeighbor2(z,x,P,appearances,R,T)

n_F= size(z,1);
n_L= (length(x) - 15) / 2;

association= ones(1,n_F) * (-1);

if n_F == 0 || n_L == 0, return, end


spsi= sin(x(9));
cpsi= cos(x(9));
zHat= zeros(2,1);
% Loop over extracted features
for i= 1:n_F
    minY= T + 1;
    
    for l= 1:n_L
        ind= (15 + (2*l-1)):(15 + 2*l);
        
        dx= x(ind(1)) - x(1);
        dy= x(ind(2)) - x(2);
        
        zHat(1)= dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi,  cpsi, spsi;
             spsi, -cpsi, -dx*cpsi - dy*spsi, -spsi, cpsi];
         
        Y= H * P([1:2,9,ind],[1:2,9,ind]) * H' + R;
        
        y2= gamma' / Y * gamma;
        
        if y2 < minY && y2 < T
            minY= y2;
            association(i)= l;
        end
    end
end


% Increase appearances counter
for i= 1:n_F
    if association(i) == -1, continue; end
    
    appearances(association(i))= appearances(association(i)) + 1;
end
end











