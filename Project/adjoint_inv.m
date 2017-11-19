function adjoint_inv(g)
    g = poseCheck(g);
    adj = LeftLiftedActionInv(g) * RightLiftedAction(g);

end