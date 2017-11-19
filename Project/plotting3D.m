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