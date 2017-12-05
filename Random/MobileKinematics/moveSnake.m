function moveSnake()
    addpath(fullfile('..','/GeoOps2D'));

    a1 = [0, 0, 1];
    h1 = [1, 0, 0];
    a_1 = [0, 0, -1];
    h_1 = [1, 0, 0];
    
    l1 = link_snake(a1, h1, 'b', h1/2);
    l_1 = link_snake(a1, h1, 'g', h_1/2);
    
    f = figure(1);
    clf(f);
    ax = axes(f);
    
    l1.linkPos(pi/2);
    l1.drawLink(ax);

end