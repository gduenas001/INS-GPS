n = 50;
XY = 10 * rand(2,n) - 5;
figure()
hold on
for i=1:n
    a= plot(XY(1,i),XY(2,i),'or','MarkerSize',5,'MarkerFaceColor','r');
    hold on
    axis([-5 5 -5 5])
    pause(.1)
    %delete(a)
end