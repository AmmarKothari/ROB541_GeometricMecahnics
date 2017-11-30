function followStraightPath()
    close(figure(1));
    f = figure(1);
    ax = axes(f);
    set(gca,'XLim',[-3 3],'YLim',[-3 3],'ZLim',[-3 3])
    xlabel('x'); ylabel('y'); zlabel('z')
    a1 = [0,0,0,0,0,1]; % rotation around z
    h1 = [1,0,0,0,0,0]; % extending in x
    a2 = [0,0,0,0,0,1]; % rotation around z
    h2 = [1,0,0,0,0,0]; % extending in x
    l1 = link(a1, h1,'b', h1/2);
    l2 = link(a2, h2,'g', h2/2);
    dt = 0.1;
    time_total = 10;
    points = 100;
    A = arm([l1, l2]);
    A = A.set_joints([pi/4,pi/4]);
    A = A.set_joint_vel([0,0]);
    A = A.calc_poses();
    A = A.calc_vels();
    start_pt = A.links(end).distal(1:3);
    end_pt = start_pt + [1, -1, 0];
    path = StraightPath(start_pt, end_pt, points);
    A.drawArm(ax);
    s = path(1, :);
    s_EE = moveToEndEffector(s, A.links(end).distal);

% 
% a = zeros(1,points);
% v = zeros(1,points);
% d = zeros(1,points);
% i = 0;
% for t = 0:dt:time_total
%     i = i + 1;
%     A = A.calcAlphaDD();
%     A = A.step(dt);
%     a(i) = A.links(1).alpha_dot_dot;
%     v(i) = A.links(1).alpha_dot;
%     d(i) = A.links(1).alpha_;
%     A.drawArm(ax);
%     A.drawArrows(ax);
%     pause(.001);
%     
%     
%     
%     
% end
% figure(2)
% plot(a, 'rx')
% hold on
% plot(v, 'bx')
% plot(d, 'cx')
% hold off

% % plotPose(ax, s, 0.5);
% view(45,45)
% pause(0.001);












end