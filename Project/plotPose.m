function plotPose(a, cosys, scale)

hold on
if nargin < 3
    scale = 0.1;
end
x1   = [1;0;0;0;0;0]*scale;
y1   = [0;1;0;0;0;0]*scale;
z1   = [0;0;1;0;0;0]*scale;
x1_moved = poseFromMatrix(leftAction(x1, cosys));
y1_moved = poseFromMatrix(leftAction(y1, cosys));
z1_moved = poseFromMatrix(leftAction(z1, cosys));
% center point of axes
scatter3(a, cosys(1), cosys(2), cosys(3))
% x axis
xQ = plotArrow(a, cosys, x1_moved-cosys, 'r');
% y axis
yQ = plotArrow(a, cosys, y1_moved-cosys, 'b');
% z axis
zQ = plotArrow(a, cosys, z1_moved-cosys, 'g');

hold off
end



function Q = plotArrow(a, cosys, pose, c)
    hold on
    Q = quiver3(a, cosys(1), cosys(2), cosys(3), pose(1), pose(2), pose(3), c);
    hold off
end