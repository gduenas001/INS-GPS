
function x= from_estimator_to_vector(obj, params)

% from cells to vectors
x= [];
for i= params.M:-1:1
    x= [x; obj.x_ph{i}];
end
x= [ x; obj.XX ];


end



