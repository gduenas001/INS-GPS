
function [idf,appearances]= nearestNeighbor(z,appearances,R,T,lm_map)

global XX PX

n_F= size(z,1);
n_L= size(lm_map,1);

idf= ones(1,n_F) * (-1);

if n_F == 0 || n_L == 0, return, end


spsi= sin(XX(9));
cpsi= cos(XX(9));
zHat= zeros(2,1);
% Loop over extracted features
for i= 1:n_F
    minY= T;
    
    for l= 1:n_L
        ind= (15 + (2*l-1)):(15 + 2*l);
        
        dx= lm_map(l,1) - XX(1);
        dy= lm_map(l,2) - XX(2);
        
        zHat(1)=  dx*cpsi + dy*spsi;
        zHat(2)= -dx*spsi + dy*cpsi;
        gamma= z(i,:)' - zHat;
        
        H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
             spsi, -cpsi, -dx*cpsi - dy*spsi];
      
        Y= H * PX([1:2,9],[1:2,9]) * H' + R;
        
        y2= gamma' / Y * gamma;
        
        if y2 < minY
            minY= y2;
            idf(i)= l;
        end
    end
end


% Increase appearances counter
for i= 1:n_F
    if idf(i) ~= -1 && idf(i) ~= 0
        appearances(idf(i))= appearances(idf(i)) + 1;
    end
end


end











