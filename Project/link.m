classdef link
    properties
        h
        h_poi
        a
        order
        pose
        zero_pose
        alpha_
        alpha_dot
        distal
        c
        % input desired velocity and output acceleration
        alpha_dot_desired
        kp
        ki
        alpha_dot_dot
        e_total
        alpha_desired
    end
    
    methods
        function obj = link(a, h, c, h_poi)
            obj.h = h;
            obj.h_poi = h_poi;
            obj.a = a;
            obj.c = c;
            obj.pose = [0,0,0,0,0,0];
            obj.zero_pose = [0,0,0,0,0,0];
            obj.kp = 1;
            obj.ki = 0;
            obj.alpha_dot = 0;
            obj.alpha_ = 0;
            obj.alpha_desired = obj.alpha_;
            obj.alpha_dot_desired = obj.alpha_dot;
            obj.alpha_dot_dot = 0;
            obj.e_total = 0;
        end
        function obj = setZero(obj, zero_pose)
            obj.zero_pose = zero_pose;
        end
        function obj = setAlphaDesired(obj, alpha_desired)
            obj.alpha_desired = alpha_desired;
        end
        function obj = calcAlphaDD_alpha(obj)
            e = obj.alpha_desired - obj.alpha_;
            obj.e_total = obj.e_total + e;
            obj.alpha_dot_dot = obj.kp * e;
            if sign(e) == sign(obj.e_total)
                obj.alpha_dot_dot = obj.alpha_dot_dot + obj.ki * obj.e_total;
            end
        end
        function obj = setAlphaDotDesired(obj, alpha_dot_desired)
            obj.alpha_dot_desired = alpha_dot_desired;
        end
        function obj = calcAlphaDD(obj)
            % calculates alpha_dd for a given error in alpha_d
            e = obj.alpha_dot_desired - obj.alpha_dot;
            obj.e_total = obj.e_total + e;
            obj.alpha_dot_dot = obj.kp * e;
            if sign(e) == sign(obj.e_total)
                obj.alpha_dot_dot = obj.alpha_dot_dot + obj.ki * obj.e_total;
%             else
%                 obj.alpha_dot_dot = obj.alpha_dot_dot + obj.ki * sign(e)*abs(obj.e_total);
            end
        end
        function obj = clearError(obj)
            obj.e_total = 0;
        end
        function obj = step(obj, dt)
            % runs system forward one time step
            obj.alpha_ = obj.alpha_ + dt*obj.alpha_dot + 1/2*obj.alpha_dot_dot*dt^2;
            obj.alpha_dot = obj.alpha_dot + dt*obj.alpha_dot_dot;
        end
        function obj = linkPos(obj, alpha_)
            obj.alpha_ = alpha_;
            % figure out how the base pose changes
            obj.pose = poseFromMatrix(rightAction(obj.zero_pose, obj.a * obj.alpha_));
            obj.distal = poseFromMatrix(rightAction(obj.pose, obj.h));
        end
        function obj = drawLink(obj, ax)
            X = [obj.pose(1), obj.distal(1)];
            Y = [obj.pose(2), obj.distal(2)];
            Z = [obj.pose(3), obj.distal(3)];
            hold on
            % plots line
            plot3(ax, X, Y, Z, obj.c, 'LineWidth', 8);
            % plots base
            plot3(ax, obj.pose(1), obj.pose(2), obj.pose(3), '^k');
            % plots distal
            plot3(ax, obj.distal(1), obj.distal(2), obj.distal(3), 'sk');
            hold off
        end
        function obj = drawDistalPose(obj, ax)
            plotPose(ax, obj.distal, 1);
        end
        
        function obj = drawPose(obj, ax, h)
            % plots a cosys at some point on link
            plotPose(ax, poseFromMatrix(rightAction(obj.pose, h)), 1);
        end
        
        function obj = drawArrow(obj, ax, arrow_params)
            poi = poseFromMatrix(rightAction(obj.pose, obj.h_poi));
            hold on;
            quiver3(ax, poi(1), poi(2), poi(3), arrow_params(1), arrow_params(2), arrow_params(3), 'linewidth',5);
            hold off;
        end
    end
    
    
    
end