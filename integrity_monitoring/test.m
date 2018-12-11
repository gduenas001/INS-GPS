clear all; clc;

n= 50000;
cell_array= cell(1,5);
new_number= rand(60,60);

tic
for i= 1:n
    cell_array= circshift(cell_array,1);
    cell_array{1}= new_number;
end
toc

% tic
% for i= 1:n
%     cell_array= [new_number, cell_array(1:end-1)];
% end
% toc
