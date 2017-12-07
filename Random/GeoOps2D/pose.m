classdef pose
    properties
        group
        p
    end
    methods
        function obj = pose(g)
            obj.group = group(g);
            obj.p = g;
        end
    end
end
    
    
    