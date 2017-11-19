syms x_delta y_delta z_delta gamma_delta beta_delta alpha_delta
syms x_zero y_zero z_zero gamma_zero beta_zero alpha_zero
P_delta = [x_delta, y_delta, z_delta, gamma_delta, beta_delta, alpha_delta];
P_zero  = [x_zero, y_zero, z_zero, gamma_zero, beta_zero, alpha_zero];
g_delta = groupSE3(P_delta);
g_zero  = groupSE3(P_zero);
d_g_delta_g_zero = [];
for i = 1:length(P_delta)
    out = diff(poseFromMatrix(rightAction(g_delta, g_zero)), P_zero(i)) ;
    d_g_delta_g_zero = [d_g_delta_g_zero, transpose(out)];
end

for i = [x_zero y_zero z_zero gamma_zero beta_zero alpha_zero]
    d_g_delta_g_zero = subs(d_g_delta_g_zero, i, 0);
end
% latex_out = latex(d_g_delta_g_zero);
% fileID = fopen('exp.tex', 'w');
% fprintf(fileID, '%s', latex_out);