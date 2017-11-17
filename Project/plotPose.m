function plotPose(cosys)

hold on
scale = 0.1;
% center point of axes
scatter3(cosys(4), cosys(5), cosys(6))
% x axis
quiver3(cosys(4), cosys(5), cosys(6), scale, 0, 0, 'r')
% y axis
quiver3(cosys(4), cosys(5), cosys(6), 0, scale, 0, 'b')
% z axis
quiver3(cosys(4), cosys(5), cosys(6), 0, 0, scale, 'g')

hold off