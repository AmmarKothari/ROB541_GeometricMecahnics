function out = group(g)
    x = g(1);
    y = g(2);
    theta = g(3);
    out = [cos(theta), -sin(theta), x;
            sin(theta), cos(theta), y;
            0, 0, 1];
end