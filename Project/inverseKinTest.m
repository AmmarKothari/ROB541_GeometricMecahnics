function inverseKinTest()
filename = 'CirclePath.gif';
RECORD = false;
frames = [];
f = figure(1);
if RECORD; f.Visible = 'off'; end
clf(f);
ax = axes(f);
view(132,16)
set(gca,'XLim',[-0.5 1.5],'YLim',[-1 3],'ZLim',[0 2.5])
xlabel('x'); ylabel('y'); zlabel('z')
grid on
% waist
a1 = [0,0,0,0,0,1]; % rotation around z
h1 = [0,0,1,0,0,0]; % extending in z
% shoulder
a2 = [0,0,0,1,0,0]; % rotation around x
h2 = [1,0,0,0,0,0]; % extending in x
% elbow
a3 = [0,0,0,0,1,0]; % rotation around y
h3 = [0,1,0,0,0,0]; % extending in y
% waist
a4 = [0,0,0,0,0,1]; % rotation around z
h4 = [0,0,1,0,0,0]; % extending in z
% shoulder 
a5 = [0,0,0,1,0,0]; % rotation around x
h5 = [1,0,0,0,0,0]; % extending in x
% elbow
a6 = [0,0,0,0,1,0]; % rotation around y
h6 = [0,1,0,0,0,0]; % extending in y

l1 = link(a1, h1,'b', h1/2);
l2 = link(a2, h2,'g', h2/2);
l3 = link(a3, h3,'r', h3/2);
l4 = link(a4, h4,'b', h4/2);
l5 = link(a5, h5,'g', h5/2);
l6 = link(a5, h5,'r', h6/2);
points = 100;
dt_dynamics = 0.01;
dt_lowlevel = 0.05;
dt_highlevel = 0.1;
dt_video = 1;
A = arm([l1, l2, l3, l4, l5, l6]);
start_alphas = [pi/4, pi/4, pi/4, pi/4, pi/4, pi/4];
A = A.set_joints(start_alphas);
A = A.set_joint_vel([0, 0, 0, 0, 0, 0]);
A = A.calc_poses();
A = A.calc_vels();
A.drawArm(ax);
start_pt = A.links(end).distal(1:3);
goal_pt = [1, 1, 1, 0, 0, 0];
goal_alphas =  A.invKin(goal_pt);
A_goal = A.set_joints(goal_alphas);
A_goal = A_goal.calc_poses();
A_goal.drawArm(ax);
path_points = 50;
travel_path = linearAlphaPath(start_alphas, goal_alphas, path_points);
plotAllPoses(A, travel_path, ax);
xlim([-3,3])
A = A.setKi(0.01); A = A.setKp(0.5);
i = 1;
i_old = 0;
i_frame = 0;
t = 0;
t_current = 0;
EE_pose = A.links(end).distal.';
EE_path = [];

while i <= length(travel_path)
    s = travel_path(i, :) * 0.1*(i-1);
    if rem(round(t, 5), dt_highlevel) < 1e-5
        % set desired joint locations
        A = A.set_alpha_desired(s);
        
    end
    
    if rem(round(t, 5), dt_lowlevel)  < 1e-5
        % update joint accelerations
        A = A.calcAlphaDD_alpha();
    end
    
    % take a step in time
    A = A.step(dt_dynamics);
    EE_pose = A.links(end).distal;
    
    % draw arm
    if rem(round(t, 5), dt_video) < 1e-5
        cla(ax);
        A.drawArm(ax);
        A.drawArrows(ax);
        EE_path = [EE_path; EE_pose];
        hold on;
        plot3(ax, travel_path(i, 1), travel_path(i, 2), travel_path(i, 3), 'ro', 'MarkerSize', 12);
        plot3(ax, travel_path(1:i,1), travel_path(1:i,2), travel_path(1:i,3), 'b');
        plot3(EE_path(:,1), EE_path(:,2), EE_path(:,3), 'c')
        hold off;
        drawnow
        i_frame = i_frame + 1;
        if RECORD; addToGif(i_frame,getframe(f), filename); end
        d = A.alpha - s;
        A_des = A.set_joints(s);
        fprintf('Point %d Distance: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f = %0.6f\n', i, d(1), d(2), d(3), d(4), d(5), d(6), norm(EE_pose(1:3) - A_des.links(end).distal(1:3)))
    
    end
    
    % check if goal has been reached
    if norm(A.alpha - s) < 0.05
        % increment to next spot in path when distance is small
        i = i + 1;
        disp('Moving to next Point')
        A = A.clearErrors();
    else
        d = A.alpha - s;
%         fprintf('Point %d Distance: %0.2f %0.2f %0.2f = %0.2f\n', i, d(1), d(2), d(3), norm(EE_pose(1:3) - s(1:3)))
    end
    
    t = t + dt_dynamics;
    t_current = t_current + dt_dynamics;
    if t_current > 50
        t_current = 0;
        i = i + 1;
        disp('Skipping Point');
    end
    
end
end