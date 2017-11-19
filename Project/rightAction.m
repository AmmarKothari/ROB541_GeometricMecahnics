function out = rightAction(h, g)
    h = groupCheck(h);
    g = groupCheck(g);   
    out = h*g;
end