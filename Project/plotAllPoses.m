function plotAllPoses(Arm, path, ax)
    % for all the poses in the path, plot the arm
    
    EE = [];
    for i = 1:5:size(path,1)
        p = path(i,:);
        A_goal = Arm.set_joints(p);
        A_goal = A_goal.calc_poses();
        A_goal.drawArm(ax);
        EE = [EE; A_goal.links(end).distal];
    end
    
    hold on;
    scatter3(EE(:,1), EE(:,2), EE(:,3), 'rx')
    hold off;
    

end