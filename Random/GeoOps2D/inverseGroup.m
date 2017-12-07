function out = inverseGroup(g)
    if all(size(g) == [1,3])
        pose = g;
    elseif all(size(g) == [3,3])
        pose = poseFromMatrix(g);
    else
        exception = MException('MyFunc:notValidSize', 'Array is not a valid size');
        throw(exception)
    end
    
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    group_T = [1, 0, -x;
            0, 1, -y;
            0, 0, 1];
    group_R = [cos(-theta), -sin(-theta), 0;
            sin(-theta), cos(-theta), 0;
            0, 0, 1];
    group_I = group_T * group_R;
    
    if all(size(g) == [1,3])
        out = poseFromMatrix(group_I);
    elseif all(size(g) == [3,3])
        out = group_I;
    end
end