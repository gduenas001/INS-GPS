function addNewLM(obj, z, R)

% Number of landmarks to add
n_L= size(z,1);

% update total number of landmarks
obj.num_landmarks= obj.num_landmarks + n_L;

% Add new landmarks to state vector
z= body2nav_3D(z,obj.XX(1:9));
zVector= z'; zVector= zVector(:);
obj.XX(end+1:end+2*n_L)= zVector;

spsi= sin(obj.XX(9));
cpsi= cos(obj.XX(9));
for i= 1:n_L
    ind= (15 + (2*i-1)):(15 + 2*i);
    
    dx= obj.XX(ind(1)) - obj.XX(1);
    dy= obj.XX(ind(2)) - obj.XX(2);
    
    H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
        spsi,  -cpsi, -dx*cpsi - dy*spsi];
    
    Y= H * obj.PX([1:2,9],[1:2,9]) * H' + R;
    
    obj.PX(end+1:end+2, end+1:end+2)= Y;
end
end