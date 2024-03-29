classdef arm
    % poses should be horizontal
    % covectors (velocity) should be vertical
    % shouldn't mater really because everything should be done on groups
    properties
        links
        alpha_dot
        alpha
        base_pose
        vs
        acc_norm_max
        vel_max
        dist_max
        alpha_dot_desired
        alpha_desired
        num_links
    end
    
    methods
        function obj = arm(links)
            obj.links = links;
            obj.num_links = length(links);
            obj.alpha_dot = zeros(length(links), 1);
            obj.alpha_dot_desired = zeros(length(links), 1);
            obj.alpha_desired = zeros(length(links), 1);
            obj.alpha = zeros(length(links), 1);
            obj.base_pose = zeros(1,6);
            obj.acc_norm_max = 10;
            obj.vel_max = 1;
            obj.dist_max = 1;
        end
        function obj = addlink(obj,link)
            obj.links = [obj.links, link];
            obj.num_links = obj.num_links + 1;
        end
        function poses = get_poses(obj)
            poses = zeros(length(obj.links)+1, 6);
            poses(1,:) = obj.base_pose; % add in the location of the base to list
            for i = 1:length(obj.links)
                poses(i,:) = obj.links(i).pose;
            end
        end
        function alphas = get_alphas(obj)
            alphas = [obj.links.alpha_];
        end
        function obj = calc_poses(obj, alphas)
            if nargin < 2
                alphas = [obj.links.alpha_];
            end
            % move first link
            obj.links(1) = obj.links(1).linkPos(alphas(1));
            for i = 2:length(obj.links)
                % update the base location of each
                % link with the distal
                % location of the previous link
                obj.links(i) = obj.links(i).setZero(obj.links(i-1).distal);
                obj.links(i) = obj.links(i).linkPos(alphas(i));
            end
        end
        function J_spatial = calc_Jacobian_spatial(obj)
            % calcualtes the jacobian using the spatial approach
            J_spatial = [];
            for j = 1:length(obj.links)
                adj_g = group_adjoint(obj.links(j).pose) * obj.links(j).a.';
                J_spatial = [J_spatial, adj_g];
            end
        end
        
        function J = J_spatial_to_world(obj, pose)
            J_spatial = obj.calc_Jacobian_spatial();
            TeRg = RightLiftedAction(pose);
            J = TeRg * J_spatial;
        end
        
        function vel_pt = calc_point_vel(obj, link_num, h_poi)
            % calculates the world velocity at a point on a link
            % link number should start at 1
            J_spatial_all = obj.calc_Jacobian_spatial(); % full jacobian
            J_spatial = J_spatial_all(:, 1:link_num); % relevant jacobian
            p = obj.links(link_num).pose; % world pose of base of link
            poi = rightAction(p, h_poi); % find the point we are interested in
            TeRg = RightLiftedAction(poi);
            J = TeRg * J_spatial; % transform into world Jacobian
            vel_pt = J * obj.alpha_dot(1:link_num); % get world velocity from joint angle velocities
        end
        
        function dtheta = JacobianPsuedoInverse(obj, ds, J)
            dtheta = JacobianPsuedoInverse(ds, J);
        end
        
        function dtheta_desired = JPIController(obj, ds, h_poi)
            ds_clamped = obj.distanceLimit(ds, obj.dist_max);
            J_spatial_all = obj.calc_Jacobian_spatial(); % full jacobian
            p = obj.links(obj.num_links).pose; % world pose of base of link
            poi = rightAction(p, h_poi); % find the point we are interested in
            TeRg = RightLiftedAction(poi);
            J = TeRg * J_spatial_all; % transform into world Jacobian
            
            % desired acceleration
            dtheta = obj.JacobianPsuedoInverse(ds_clamped,J);
            dtheta_desired = dtheta;
        end
        
        function obj = step(obj, dt)
            for i = 1:length(obj.links)
                obj.links(i) = obj.links(i).step(dt);
                obj.alpha(i) = obj.links(i).alpha_;
                obj.alpha_dot(i) = obj.links(i).alpha_dot;
            end
            obj = obj.calc_poses();
            obj = obj.calc_vels();
        end
        
        function ddtheta = accelerationLimit(obj, ddtheta, acc_max)
            % clamp norm to a max value
            if norm(ddtheta) > acc_max
                ddtheta = ddtheta / norm(ddtheta) * acc_max;
            end
        end
        
        function ds_clamped = distanceLimit(obj, ds, dist_max)
            if norm(ds) > dist_max
                ds_clamped = ds / norm(ds) * dist_max;
            else
                ds_clamped = ds;
            end
        end
        
        function obj = calc_vels(obj)
            % calculates velocities for the current alpha and alpha_dot
            vs = [];
            for i = 1:obj.num_links
                vel_pt = double(obj.calc_point_vel(i, obj.links(i).h_poi)).';
                vs = [vs; vel_pt];
            end
            obj.vs = vs;
        end
        
        function obj = set_joint_vel(obj, alpha_dot)
            for i = 1:obj.num_links
                obj.links(i).alpha_dot = alpha_dot(i);
            end
        end
        
        function obj = set_joints(obj, alpha_)
            for i = 1:obj.num_links
                obj.links(i).alpha_ = alpha_(i);
            end
        end
        
        function obj = set_alpha_desired(obj, alphas)
            obj.alpha_desired = alphas;
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).setAlphaDesired(alphas(i));
            end
        end
        
        function obj = set_alpha_dot_desired(obj, vs_des)
            obj.alpha_dot_desired = vs_des;
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).setAlphaDotDesired(vs_des(i));
            end
        end
        
        function obj = calcAlphaDD(obj)
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).calcAlphaDD();
            end
        end
        
        function obj = calcAlphaDD_alpha(obj)
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).calcAlphaDD_alpha();
            end
        end
        
        function obj = setKp(obj, kp)
            for i = 1:obj.num_links
                obj.links(i).kp = kp;
            end
        end
        
        function obj = setKi(obj, ki)
            for i = 1:obj.num_links
                obj.links(i).ki = ki;
            end
        end
        
        function obj = clearErrors(obj)
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).clearError();
            end
        end
        
        function obj = drawArm(obj, ax)
            for i = 1:obj.num_links
                obj.links(i).drawLink(ax);
            end
        end
        
        function obj = drawArrows(obj, ax)
            for i = 1:obj.num_links
                obj.links(i).drawArrow(ax, obj.vs(i,:));
            end
        end
        
        function obj = drawDistalPose(obj, ax, link_num)
            obj.links(link_num).drawDistalPose(ax);
        end
        
        function dist = dist_to_goal(obj, goal, al)
            f_arm = obj.calc_poses(al);
            dist = f_arm.links(end).distal - goal;
        end
        
        function dist = dist_to_pose(obj, pose, min_alphas)
            dist = pose - min_alphas;
        end
        
        function goal_alphas = invKin(obj, goal, min_alphas)
            cart_func = @(al) sum(0.5 * obj.dist_to_goal(goal, al).^2);
            alpha_func = @(al) sum(0.5 * obj.dist_to_pose(al, min_alphas).^2); 
            w = [1, 0.1];
            dist_func = @(al) w(1)*cart_func(al) + w(2)*alpha_func(al);
            x0 = obj.get_alphas();
            % add joint constraints!
            ub = pi*ones(size(min_alphas));
            lb = -pi*ones(size(min_alphas));
            goal_alphas = fmincon(dist_func, x0, [], [], [], [], lb, ub);
        end
    end
    
end