
function [x,P]= addNewLM2(z,x,P,R)

% Number of landmarks to add
n_L= size(z,1);

% Add new landmarks to state vector
z= body2nav(z,x(1:9));
x(end+1:end+2*n_L)= z;


spsi= sin(x(9));
cpsi= cos(x(9));
for i= 1:n_L
    ind= (15 + (2*i-1)):(15 + 2*i);

    dx= x(ind(1)) - x(1);
    dy= x(ind(2)) - x(2);
    
    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
        spsi,  -cpsi, -dx*cpsi - dy*spsi];
    
    Y= H * P([1:2,9],[1:2,9]) * H' + R;
    
    P(end+1:end+2, end+1:end+2)= Y;
end



