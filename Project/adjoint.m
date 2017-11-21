function out = adjoint(g)
    g = poseCheck(g);
    out = RightLiftedActionInv(g) * LeftLiftedAction(g);
end