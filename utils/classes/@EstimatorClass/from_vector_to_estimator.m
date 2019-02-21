function from_vector_to_estimator(obj, x, params)

% from a vector to cells 
inds= 1:params.m;
for i= params.M:-1:1
    % update the cell
    obj.x_ph{i}= x(inds);
    
    % update index
    inds= inds + params.m;
end
% current pose
obj.XX= x(end - params.m + 1:end);


end
