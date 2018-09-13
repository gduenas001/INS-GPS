
function zBody= removeFeatureInArea(x, zBody, minX, maxX, minY, maxY)

zNav= body2nav(zBody,x);

% Remove people-features
inX= (zNav(:,1) > minX) & (zNav(:,1) < maxX);
inY= (zNav(:,2) > minY) & (zNav(:,2) < maxY);

zBody( inX & inY, :)= [];

end



function z= body2nav(z,x)

% Convert in 3D
R_NB= R_NB_rot(x(7),x(8),x(9));
z= ( R_NB * z' + x(1:3) )';

% % Convert in 2D
% z= z(:,1:2);
% R_NB= R_NB_rot(0,0,x(9));
% R_NB= R_NB(1:2,1:2);
% z= ( R_NB * z' + x(1:2) )';

% Put the back into 2D
z= z(:,1:2);

end