
function plot_attitude(x, figHandle)

xPlot= [-0.3; 0; -0.3];
yPlot= [0.1; 0; -0.1];
zPlot= [0; 0; 0];
xyz_B= [xPlot, yPlot, zPlot]';

figure(figHandle); hold on;

for i= 1:size(x,2)
    
    R_NB= R_NB_rot(x(7,i),x(8,i),x(9,i));
    xyz_N= R_NB*xyz_B + x(1:3,i);
    
    plot3(xyz_N(1,:), xyz_N(2,:), xyz_N(3,:), 'g-', 'linewidth', 2);
    
    
end


















