classdef arm
    properties
        links
    end
    
    methods
        function obj = arm(links)
            obj.links = links;
        end
        function obj = addlink(obj,link)
            obj.links = [obj.links, link];
        end
        function obj = moveArm(obj, alphas)
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