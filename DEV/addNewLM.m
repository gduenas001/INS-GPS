
function [x,P]= addNewLM(z,x,P,R)

% Number of landmarks to add
n_L= size(z,1);

for i= 1:n_L
    x(end+1:end+2)= z(i,:)';
    P(end+1:end+2,end+1:end+2)= P(1:2,1:2) + R;
end












