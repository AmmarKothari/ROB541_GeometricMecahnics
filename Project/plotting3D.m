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
f = figure(1);
ax = axes(f);
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,1,0,0,0,0]; % extending in y
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,0,1,0,0,0]; % extending in z
a4 = [0,0,0,0,1,0]; % rotation around y
h4 = [0,0,1,0,0,0]; % extending in z
l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'r', h2/2);
l3 = link(a3, h3,'g', h3/2);
l4 = link(a4, h4,'c', h4/2);

A = arm([l1, l2, l3, l4]);
view(45,45)
set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
xlabel('x'); ylabel('y'); zlabel('z')
p = zeros(length(A.links));
for il = 1:length(A.links)
    for i = 0:0.1:pi
        p(il) = p(il) + 0.1;
    %     A = A.set_joints([i/2, i, -i]);
        A = A.set_joints(p);
        A = A.calc_poses();
    %     A = A.calc_vels();
        cla(ax)
        A.drawArm(ax);
        A.drawDistalPose(ax, 1);
        A.links(1).drawPose(ax, A.links(1).h_poi);
        A.links(2).drawPose(ax, A.links(2).h_poi);
        A.links(3).drawPose(ax, A.links(3).h_poi);
        pause(0.05)
    end
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
    cla(ax);
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
    A = A.calc_vels();
    cla(ax)
    A.drawArm(ax);
    A.drawArrows(ax);
    
    t_old = t;
    pause(0.0001);
end
% A.J_spatial()
%%
% Testing out low level controller for links
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [1,0,0,0,0,0]; % extending in x
l1 = link(a1, h1,'b', h1/2);
close(figure(1));
f = figure(1);
ax = axes(f);
l1 = l1.setAlphaDotDesired(0.1);
dt = 0.01;
points = 1000;
a = zeros(1,points);
v = zeros(1,points);
d = zeros(1,points);
for i = 1:points
    l1 = l1.calcAlphaDD();
    l1.alpha_dot = l1.alpha_dot + dt * l1.alpha_dot_dot;
    a(i) = l1.alpha_dot_dot;
    v(i) = l1.alpha_dot;
    d(i) = l1.alpha_ + l1.alpha_dot * dt;
    l1 = l1.linkPos(d(i));
    l1.drawLink(ax);
    pause(0.0001);
end
figure(2)
plot(v, 'rx')
hold on
plot(a, 'bo')
hold off

%%
% Testing out simple controller with straight line path
% close(figure(1));
% f = figure(1);
% ax = axes(f);
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [1,0,0,0,0,0]; % extending in x
a2 = [0,0,0,0,0,1]; % rotation around z
h2 = [1,0,0,0,0,0]; % extending in x
l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'g', h2/2);
dt = 0.01;
points = 1000;
A = arm([l1, l2]);
A = A.set_joints([pi/4,pi/4]);
A = A.set_joint_vel([0,0]);
A = A.calc_poses();
A = A.calc_vels();


A = A.set_alpha_dot_desired([0.1, 0.1]);
for i = 1:points
    A = A.calcAlphaDD();
    
    
    
end

% A.drawArm(ax);
% A.drawArrows(ax);
% % plotPose(ax, s, 0.5);
% view(45,45)
% set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
% xlabel('x'); ylabel('y'); zlabel('z')
% pause(0.001);









