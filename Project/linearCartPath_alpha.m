function alpha_points = linearCartPath_alpha(Arm, start_alpha, goal_cart, pts)
% attempts to create a path through cartesian space that is a straight line
% in joint angles
    sub_points = ceil(pts/10);
    A_start = Arm.calc_poses(start_alpha);
    start_cart = A_start.links(end).distal;
    c = linearAlphaPath(start_cart, goal_cart, sub_points);
%     a_init = acos(1-2*rand(sub_points,Arm.num_links)); % randomly sample angles from 0 to 2*pi
%     a_init = ones(sub_points, 1) * start_alpha; % initial guess for each point is starting pose
%     func = @(a) cartErrorPath(Arm, a, c, start_alpha);
%     
%     ub = [pi, pi, pi, pi, pi, pi];
%     lb = [-pi, -pi, -pi, -pi, -pi, -pi];
%     opts = optimoptions(@fmincon, 'Display', 'iter');
%     alpha_points_coarse = fmincon(func, a_init, [], [], [], [], lb, ub, [], opts);
    alpha_points_coarse = [];
    for i = 1:size(c,1)
        alpha_points_coarse = [alpha_points_coarse; Arm.invKin(c(i,:), start_alpha)];
    end
    alpha_points = [];
    for i = 1:size(alpha_points_coarse,1)-1
        alpha_points = [alpha_points; linearAlphaPath(alpha_points_coarse(i,:), alpha_points_coarse(i+1,:), ceil(pts/sub_points))];
    end
end

function error = cartErrorPoint(Arm, a, c, min_pose)
    A = Arm.calc_poses(a);
    a_c = A.links(end).distal;
    cart_error = sum(0.5*(c - a_c).^2); % distance from point in cartesian space
     % distance in joint space from min_pose
     % should force similar positions to be near each other.
    joint_error = sum(0.5*(a - min_pose).^2);
    
    error = cart_error + joint_error;
end

function error = cartErrorPath(Arm, as, cs, min_pose)
    errors = zeros(size(as,1),1);
    parfor i = 1:size(as,1)
        errors(i) = cartErrorPoint(Arm, as(i,:), cs(i,:), min_pose);
    end
    error = sum(errors);
end