function addNewLM(obj, z, R)

% Number of landmarks to add
n_L= size(z,1);

% update total number of landmarks
%obj.num_landmarks= obj.num_landmarks + n_L;
obj.num_of_estimated_landmarks = obj.num_of_estimated_landmarks + n_L;

% Add new landmarks to state vector
z_global= body2nav_2D(z,obj.XX(1:2),obj.XX(3));
zVector= z_global'; zVector= zVector(:);
obj.XX(end+1:end+2*n_L)= zVector;

spsi= sin(obj.XX(3));
cpsi= cos(obj.XX(3));
for i= 1:n_L
    ind= (3 + (2*i-1)):(3 + 2*i);
    
    dx= obj.XX(ind(1)) - obj.XX(1);
    dy= obj.XX(ind(2)) - obj.XX(2);
    
    % H= [-cpsi, -spsi, -dx*spsi + dy*cpsi;
    %     spsi,  -cpsi, -dx*cpsi - dy*spsi];
    
    % Y= H * obj.PX(1:3,1:3) * H' + R;
    J_X = [1, 0, z(i,:)*[-spsi; -cpsi];
         0, 1, z(i,:)*[ cpsi; -spsi]];
    J_V = [cpsi, -spsi;
           spsi,  cpsi];
    
    Y= J_X * obj.PX(1:3,1:3) * J_X' + J_V * R * J_V';
    
    obj.PX(end+1:end+2, end+1:end+2)= Y;
    %obj.PX(end-1:end, 1:3)= J_X * obj.PX(1:3,1:3);
    %obj.PX(1:3, end-1:end)= obj.PX(1:3,1:3) * J_X';
end
end