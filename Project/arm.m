classdef arm
    properties
        links
        alpha_dot
        alpha
        base_pose
        vs
    end
    
    methods
        function obj = arm(links)
            obj.links = links;
            obj.alpha_dot = zeros(length(links), 1);
            obj.alpha = zeros(length(links), 1);
            obj.base_pose = zeros(1,6);
        end
        function obj = addlink(obj,link)
            obj.links = [obj.links, link];
            obj.alpha_dot = [obj.alpha_dot; 0];
            obj.alpha = [obj.alpha; 0];
        end
        function obj = calc_poses(obj, alphas)
            if nargin < 2
                alphas = obj.alpha;
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
                disp(j)
                adj_g = adjoint(obj.links(j).pose) * obj.links(j).a.';
                J_spatial = [J_spatial, adj_g];
            end
        end
        
        function J = J_spatial_to_world(obj, pose)
            J_spatial = obj.calc_Jacobian_spatial();
            TeRg = RightLiftedAction(pose);
            J = TeRg * J_spatial;
        end
        
        function poses = get_poses(obj)
            poses = zeros(length(obj.links)+1, 6);
            poses(1,:) = obj.base_pose; % add in the location of the base to list
            for i = 1:length(obj.links)
                poses(i,:) = obj.links(i).pose;
            end
        end
        
        function vel_pt = calc_point_vel(obj, link_num, h_poi)
            % calculates the world velocity at a point on a link
            % link number should start at 1
            J_spatial_all = obj.calc_Jacobian_spatial(); % full jacobian
            J_spatial = J_spatial_all(:, 1:link_num); % relevant jacobian
            p = obj.links(link_num+1).pose; % pose of base of link?
            % below isn't done in the best way
            poi = rightAction(p, inverseGroup(h_poi)); % find the point we are interested in
            TeRg = RightLiftedAction(poi);
            J = TeRg * J_spatial; % transform into world Jacobian
            vel_pt = J * obj.alpha_dot(1:link_num); % get world velocity from joint angle velocities
        end
        
        function obj = move(obj)
            vs = [];
            poses = obj.get_poses();
            for i = 1:length(obj.links)
                p_prox = poses(i,:);
                p_dist = poses(i+1,:);
                vel_pt = double(obj.calc_point_vel(i, obj.links(i).h_poi));
                vs = [vs; vel_pt];
            end
            obj.vs = vs;
        end
        
        function obj = set_joint_vel(obj, alpha_dot)
            obj.alpha_dot = alpha_dot;
        end
        
        function obj = set_joints(obj, alpha)
            obj.alpha = alpha;
        end
        
        function obj = drawArm(obj, ax)
            for i = 1:length(obj.links)
                obj.links(i).drawLink(ax);
            end
        end
        
        function obj = drawDistalPose(obj, ax, link_num)
            obj.links(link_num).drawDistalPose(ax);
        end
    end
    
end