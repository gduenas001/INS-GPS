
function addNewLM(z,R)

global XX PX

% Number of landmarks to add
n_L= size(z,1);

% Add new landmarks to state vector
z= body2nav(z,XX(1:9));
zVector= z'; zVector= zVector(:);
XX(end+1:end+2*n_L)= zVector;


spsi= sin(XX(9));
cpsi= cos(XX(9));
for i= 1:n_L
    ind= (15 + (2*i-1)):(15 + 2*i);

    dx= XX(ind(1)) - XX(1);
    dy= XX(ind(2)) - XX(2);
    
    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
        spsi,  -cpsi, -dx*cpsi - dy*spsi];
    
    Y= H * PX([1:2,9],[1:2,9]) * H' + R;
    
    PX(end+1:end+2, end+1:end+2)= Y;
end



