function path = SpiralPath(angle_range, R, a)
    x = zeros(length(angle_range),1);
    y = zeros(length(angle_range),1);
    z = zeros(length(angle_range),1);
    orientation = zeros(length(angle_range), 3);
    
    for i1 = 1:length(angle_range)
       x(i1) = R * cos(angle_range(i1));
       y(i1) = R * sin(angle_range(i1));
       z(i1) = a * angle_range(i1);
    end
    path = [x, y, z, orientation];
end