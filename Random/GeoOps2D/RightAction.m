function out = RightAction(g,h)
    g_group = group(g);
    h_group = group(h);
    out = g_group * h_group;
end
