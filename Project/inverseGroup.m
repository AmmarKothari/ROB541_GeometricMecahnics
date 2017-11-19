function out = inverseGroup(pose)
    % check to convert into pose representation if it is not
    if size(pose) == [4,4]
        pose = poseFromMatrix(pose);
    end
    % page 37 in sastry
    group_inv = groupSE3(-pose, 'y');
%     pose_inv = poseFromMatrix(group_inv);
    out = group_inv;

end