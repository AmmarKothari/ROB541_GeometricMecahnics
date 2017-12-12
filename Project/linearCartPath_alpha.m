function alpha_points = linearCartPath_alpha(Arm, start_cart, goal_cart, pts)
% attempts to create a path through cartesian space that is a straight line
% in joint angles
    sub_points = ceil(pts/10);
    c = linearAlphaPath(start_cart, goal_cart, sub_points);
    a_init = acos(1-2*rand(sub_points,Arm.num_links)); % randomly sample angles from 0 to 2*pi
    
    func = @(a) cartErrorPath(Arm, a, c);
    ub = [pi, pi, pi, pi, pi, pi];
    lb = [-pi, -pi, -pi, -pi, -pi, -pi];
    opts = optimoptions(@fmincon, 'Display', 'iter');
    alpha_points = fmincon(func, a_init, [], [], [], [], lb, ub, [], opts);
end

function error = cartErrorPoint(Arm, a, c)
    A = Arm.calc_poses(a);
    a_c = A.links(end).distal;
    error = sum(0.5*(c - a_c).^2);
end

function error = cartErrorPath(Arm, as, cs)
    errors = zeros(size(as,1),1);
    parfor i = 1:size(as,1)
        errors(i) = cartErrorPoint(Arm, as(i,:), cs(i,:));
    end
    error = sum(errors);
end