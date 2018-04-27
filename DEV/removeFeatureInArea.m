
function z= removeFeatureInArea(z, minX, maxX, minY, maxY)

% Remove people-features
inX= (z(:,1) > minX) & (z(:,1) < maxX);
inY= (z(:,2) > minY) & (z(:,2) < maxY);

z( inX & inY, :)= [];

















