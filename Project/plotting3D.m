f = figure(1);
a = axes(f);
pose = [1,0,0,0,0,0];
g = groupSE3(pose);
view(45, 45)
for p = [4, 5, 6]
    for i = 0:0.1:1
        pose(p) = pose(p) + 0.1;
        plotPose(a, pose);
        pause(0.1)
    end
end
%%
close(figure(1));
f = figure(1);
ax = axes(f);
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,0,1,0,0,0]; % extending in z
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,0,1,0,0,0]; % extending in z
l1 = link(a1, h1,'b');
l2 = link(a2, h2,'r');
l3 = link(a3, h3,'g');

A = arm([l1, l2, l3]);
view(45,45)
set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
xlabel('x'); ylabel('y'); zlabel('z')
for i = 0:0.1:pi
    A = A.moveArm([i/2, i, -i]);
    A.drawArm(ax);
    A.drawDistalPose(ax, 3);
    pause(0.25)
    cla(ax)
end

%%
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,0,1,0,0,0]; % extending in z
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,0,1,0,0,0]; % extending in z
l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'r', h2/2);
l3 = link(a3, h3,'g', h3/2);
% 
% A = arm([l1, l2, l3]);
% simple arm
% a1 = [0,0,0,0,0,1]; % rotation around z
% h1 = [1,0,0,0,0,0]; % extending in z
% a2 = [0,0,0,0,0,1]; % rotation around Z
% h2 = [1,0,0,0,0,0]; % extending in z
% a3 = [0,0,0,0,0,1]; % rotation around z
% h3 = [1,0,0,0,0,0]; % extending in z
% l1 = link(a1, h1,'b', h1/2);
% l2 = link(a2, h2,'r', h2/2);
% l3 = link(a3, h3,'g', h3/2);

A = arm([l1, l2, l3]);
close(figure(1));
f = figure(1);
ax = axes(f);
view(45,45)
set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
xlabel('x'); ylabel('y'); zlabel('z')

% Simple velocity vector test
alpha_dot = [pi/5, 0, 0];
joint_alpha = [0,0,0];
A.set_joints(joint_alpha);
A = A.set_joint_vel(alpha_dot);
A = A.calc_poses();
A = A.calc_vels();
A.drawArm(ax);
A.drawArrows(ax);


%%
dt = 0.1;
time_traj = 0:dt:10;
alpha_dot_traj = zeros(length(time_traj), length(A.links));
joint_alpha = zeros(length(time_traj)+1, length(A.links));
for i = 1:length(time_traj)
    alpha_dot_traj(i,:) = [pi/5, 0, 0];
    joint_alpha(i+1,:) = joint_alpha(i,:) + dt * alpha_dot_traj(i,:);
    A = A.set_joints(joint_alpha(i,:));
    A  = A.calc_poses();
    A.drawArm(ax);
    pause(0.1);
end
t_old = 0;
%%
joint_alpha = [0,0,0];
for i = 1:length(time_traj)
    t = time_traj(i);
    alpha_dot = alpha_dot_traj(i, :);
    dt = t - t_old;
    joint_alpha = joint_alpha + dt * alpha_dot;
    A = A.set_joints(joint_alpha);
    A = A.set_joint_vel(alpha_dot);
    A = A.calc_poses();
    A = A.move();
    A.drawArm(ax);
    A.drawArrows(ax);
    
    t_old = t;
    pause(0.1);
end
% A.J_spatial()
%%
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
A = A.set_joints([0,0,0]);
A = A.set_joint_vel([0,0,0]);
A = A.calc_poses();
A = A.calc_vels();
view(45,45)
set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
xlabel('x'); ylabel('y'); zlabel('z')

travel_path = SpiralPath(0:0.1:4*pi, 2, 1);
i = 1;
EE_pose = A.links(end).distal.';
while i <=length(travel_path)
    s = travel_path(i, :);
    ds = moveToEndEffector(s, EE_pose).';
    % calculates Jacobianfigures out control value
    % and sends that value (probably want to split this up a bit)
    dt = 0.01;
    A = A.JPIController(ds, EE_pose, dt);
    EE_pose = A.links(end).distal;
    if norm(EE_pose(1:3) - s(1:3)) < 0.5
        % increment to next spot in path when distance is small
        i = i + 1;
        disp('Moving to next Point')
    else
        d = EE_pose(1:3) - s(1:3);
        sprintf('Point %d Distance: %0.2f %0.2f %0.2f', i, d(1), d(2), d(3))
    end
end

