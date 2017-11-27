function p_out = moveToEndEffector(pose, ee_pose)
    % moves the given point into the coordinate frame of the end effector
    ee_pose = groupCheck(ee_pose);
    pose = groupCheck(pose);
    h = inverseGroup(ee_pose) * pose;
    p_out = poseFromMatrix(h);
end