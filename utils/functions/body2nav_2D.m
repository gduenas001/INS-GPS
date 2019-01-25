
function z= body2nav_2D(z, x, yaw)

if isempty(z), return, end

% Convert in 2D
z= z(:,1:2);
R_NB= R_NB_rot(0, 0, yaw);
R_NB= R_NB(1:2,1:2);
z= ( R_NB * z' + x(1:2) )';
