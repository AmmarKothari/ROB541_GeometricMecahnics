function adjoint(g)
    g = poseCheck(g);
    adj = RightLiftedActionInv(g) * LeftLiftedAction(g);

end