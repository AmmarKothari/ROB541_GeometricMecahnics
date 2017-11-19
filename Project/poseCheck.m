function out = poseCheck(group)
% check to convert into pose representation if it is not
    if size(group) == [4,4]
        out = groupSE3(group);
    else
        out = group;
    end

end