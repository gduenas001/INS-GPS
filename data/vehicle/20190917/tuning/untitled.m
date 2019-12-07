%plot(estimator.landmark_map(:,1),estimator.landmark_map(:,2),'b.')
load('FG.mat');

tmp = inf*ones(size(data.pose,1)-1,9);
for i = 1:size(tmp,1)
    tmp(i,:) = data.pose{i};
end

plot(tmp(:,1),tmp(:,2));
hold on
for i = 1:1999
    if isempty(data.lidar{i})
        continue;
    end
    pose=data.pose{i};
    z=data.lidar{i};
    b=0;

    a=[cos(pose(9)-b),-sin(pose(9)-b);sin(pose(9)-b),cos(pose(9)-b)]*(z');%*c;
    poseXY=[pose(1);pose(2)];
    a(1,:)=a(1,:)+poseXY(1);
    a(2,:)=a(2,:)+poseXY(2);

    plot(poseXY(1),poseXY(2),'r.')
    if ~isempty(a)
        plot(a(1,:),a(2,:),'k.')
    end
end
axis equal