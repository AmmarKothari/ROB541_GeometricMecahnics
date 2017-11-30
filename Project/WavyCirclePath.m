function path = WavyCirclePath(start_pt, angle_range, R, z_func)
    c = start_pt;
    c(1) = c(1) - R;
    x = zeros(length(angle_range),1);
    y = zeros(length(angle_range),1);
    z = zeros(length(angle_range),1);
    orientation = zeros(length(angle_range), 3);
    
    for i1 = 1:length(angle_range)
       x(i1) = c(1) + R * cos(angle_range(i1));
       y(i1) = c(2) + R * sin(angle_range(i1));
       z(i1) = c(3) + z_func(4*angle_range(i1))/10;
    end
    path = [x, y, z, orientation];
end