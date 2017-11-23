function out = LiftedAction(side, g, inverse)
    syms x_delta y_delta z_delta gamma_delta beta_delta alpha_delta
    syms x_zero y_zero z_zero gamma_zero beta_zero alpha_zero
    P_delta = [x_delta, y_delta, z_delta, gamma_delta, beta_delta, alpha_delta];
    P_zero  = [x_zero, y_zero, z_zero, gamma_zero, beta_zero, alpha_zero];
    g_delta = groupSE3(P_delta);
    g_zero  = groupSE3(P_zero);
    if side == 'r'
        p_g_delta_g_zero = poseFromMatrix(rightAction(g_zero, g_delta));
    elseif side == 'l'
        p_g_delta_g_zero = poseFromMatrix(leftAction(g_zero, g_delta));
    end
    d_g_delta_g_zero = [];
    % d(g_delta * g_zero)/d(g_zero)
    for i = 1:length(P_delta)
        out = diff(p_g_delta_g_zero, P_zero(i)) ;
        d_g_delta_g_zero = [d_g_delta_g_zero, transpose(out)];
    end
    
    % evaluate g_zero at the identity (which is [0,0,0,0,0,0])
    for i = [x_zero y_zero z_zero gamma_zero beta_zero alpha_zero]
        d_g_delta_g_zero = subs(d_g_delta_g_zero, i, 0);
    end

    % evaluate g_delta at g
    out = d_g_delta_g_zero;
    g = poseCheck(g);
    for i = 1:length(P_delta)
        out = subs(out, P_delta(i), g(i));
    end
    
    % check inverse
    if inverse == 'y'
        out = inv(out);
    end
end