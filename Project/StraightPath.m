function path = StraightPath(startpt, endpt, points)
    % linearly interpolate between start and end point
    x = linspace(startpt(1), endpt(1), points + 1).';
    y = linspace(startpt(2), endpt(2), points + 1).';
    z = linspace(startpt(3), endpt(3), points + 1).';
    orientation = zeros(points+1, 3);
    path = [x, y, z, orientation];
end