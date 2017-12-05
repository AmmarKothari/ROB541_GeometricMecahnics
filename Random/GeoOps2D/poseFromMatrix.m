function g = poseFromMatrix(G)
    g = zeros(1,3);
    g(1) = G(1,3);
    g(2) = G(2,3);
    g(3) = atan2(G(2,1), G(1,1));
end
