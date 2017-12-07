function moveSnake()
    addpath(fullfile('..','/GeoOps2D'));

    a0 = [0, 0, 1];
    h0 = [1, 0, 0]/2;
    a1 = [0, 0, 1];
    h1 = [1, 0, 0]/2;
    a_1 = [0, 0, -1];
    h_1 = [-1, 0, 0]/2;
    
    l0 = link_snake(a0, h0, 'c', h0/2);
    l1 = link_snake(a1, h1, 'r', h1/2);
    l_1 = link_snake(a_1, h1, 'g', h_1/2);
    
    f = figure(1);
    clf(f);
    ax = axes(f);
    
    b = body([l_1, l0, l1], 2);
    b = b.calc_poses([-pi/8, pi/4]);
    b.links(1).drawLink(ax);
    b.links(2).drawLink(ax);
    b.links(3).drawLink(ax);
%     l1 = l1.linkPos(0);
%     l1.drawLink(ax);
%     l_1 = l_1.setZero(l1.distal);
%     l_1 = l_1.linkPos(pi/4);
%     l_1.drawLink(ax);

end