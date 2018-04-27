
function [association]= nearestNeighbor(z,x,P,R,T)

n_F= size(z,1);
n_L= (length(x) - 15) / 2;

association= ones(1,n_F) * (-1);

if n_F == 0 || n_L == 0, return, end



zNew= [];
for i= 1:n_F
    minY= T + 1;
    
    for l= 1:n_L
        ind= (15 + (2*l-1)):(15 + 2*l);
        gamma= z(i,:)' - x(ind);
        Y= P(1:2,1:2) + P(ind,ind) + R;  % this does not depend on the uncertainty in the yaw -- that's wrong
        y2= gamma' / Y * gamma;
        
        if y2 < minY && y2 < T
            minY= y2;
            association(i)= l;
        end
    end
end






















