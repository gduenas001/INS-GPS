plot(estimator.landmark_map(:,1),estimator.landmark_map(:,2),'b.')
hold on
for i = 1:length(FG.lidar)
    if isempty(FG.lidar{i})
        continue;
    end
    pose=FG.pose{i};
    z=FG.lidar{i};
    b=0;
    %pose(9) =pose(9)*pi/180;
    %psi_offset=-90*pi/(180);
    %c=[cos(psi_offset),-sin(psi_offset);sin(psi_offset),cos(psi_offset)]*([z(:,1),z(:,2)]');
    a=[cos(pose(9)-b),-sin(pose(9)-b);sin(pose(9)-b),cos(pose(9)-b)]*(z');%*c;
    poseXY=[pose(1);pose(2)];
    a(1,:)=a(1,:)+poseXY(1);
    a(2,:)=a(2,:)+poseXY(2);
    %ang_deg= -1.5;
    %tran_offset=[0;0.0];
    %rot_offset= [cos(ang_deg*pi/180),sin(ang_deg*pi/180);-sin(ang_deg*pi/180),cos(ang_deg*pi/180)];
    %poseXY=rot_offset*poseXY +tran_offset;
    %a=rot_offset*a +tran_offset;
    plot(poseXY(1),poseXY(2),'r.')
    if ~isempty(a)
        plot(a(1,:),a(2,:),'k.')
    end
end
axis equal