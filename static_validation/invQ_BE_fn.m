
function [invQ_BE]= invQ_BE_fn(phi,theta)

invQ_BE= [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi), -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

