hessx = @(x) -sum(abs(diag(sqrtm(obj.PX)*[0 0 sin(x(3)); 0 0 -cos(x(3)); sin(x(3)) -cos(x(3)) -(obj.landmark_map(obj.association_no_zeros(i),1)-x(1))*cos(x(3))-(obj.landmark_map(obj.association_no_zeros(i),2)-x(2))*sin(x(3))]*sqrtm(obj.PX))));
min = [X_range(1,1),X_range(1,2),X_range(1,3)];
max = [X_range(2,1),X_range(2,2),X_range(2,3)];
input = [];
output = [];
X= linspace(X_range(1,1),X_range(2,1),100);
Y= linspace(X_range(1,2),X_range(2,2),100);
Z= linspace(X_range(1,3),X_range(2,3),100);
for i = 1:100
    for j = 1:100
        for k = 1:100
            input = [input, [X(i);Y(j);Z(k)] ];
            output = [output, hessx([X(i);Y(j);Z(k)]) ];
        end
    end
end