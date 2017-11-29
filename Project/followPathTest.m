function followPathTest()


close(figure(1));
f = figure(1);
ax = axes(f);
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,0,1,0,0,0]; % extending in z
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,0,1,0,0,0]; % extending in z
l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'r', h2/2);
l3 = link(a3, h3,'g', h3/2);

A = arm([l1, l2, l3]);
A = A.set_joints([pi/4,pi/4,pi/4]);
A = A.set_joint_vel([0,0,0]);
A = A.calc_poses();
A = A.calc_vels();
view(45,45)
set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
xlabel('x'); ylabel('y'); zlabel('z')

% travel_path = SpiralPath(0:pi/4:4*pi, 1, 0.5);
travel_path = SpiralPath(0:pi/4:4*pi, 1, 0); %flat circle
travel_path(:,3) = travel_path(:,3) + 2;
i = 1;
t = 0;
dt = 0.1;
controller_dt = 1;
while i <=length(travel_path)
    t = t + dt;
    if i == 2
        i = 2;
    end
    s = travel_path(i, :);
%     s = [1.1, 0, 1.9, 0, 0, 0];
    EE_pose = A.links(end).distal.';
    ds = moveToEndEffector(s, EE_pose).';
    % calculates Jacobianfigures out control value
    % and sends that value (probably want to split this up a bit)
    if abs((mod(t, controller_dt) - controller_dt)) < 1e-5
        A = A.JPIController(ds, EE_pose, dt);
    end
    EE_pose = A.links(end).distal;
    if norm(EE_pose(1:3) - s(1:3)) < 0.1
        % increment to next spot in path when distance is small
        i = i + 1;
        disp('Moving to next Point')
    else
        d = EE_pose(1:3) - s(1:3);
        fprintf('Point %d Distance: %0.2f %0.2f %0.2f = %0.2f\n', i, d(1), d(2), d(3), norm(EE_pose(1:3) - s(1:3)))
    end
%     cla(ax);
%     plot3(travel_path(:,1), travel_path(:,2), travel_path(:,3));
    A.drawArm(ax);
    A.drawArrows(ax);
    plotPose(ax, s, 0.5);
    view(45,45)
    set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
    xlabel('x'); ylabel('y'); zlabel('z')
    pause(0.001);


end

end