function out = adjoint_inv(g)
    g = poseCheck(g);
    out = LeftLiftedActionInv(g) * RightLiftedAction(g);

end