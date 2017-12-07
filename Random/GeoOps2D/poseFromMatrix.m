function g = poseFromMatrix(G)
    if all(size(G) == [3,3])
        g = zeros(1,3);
        g(1) = G(1,3);
        g(2) = G(2,3);
        g(3) = atan2(G(2,1), G(1,1));
    elseif all(size(G) == [1,3])
        g = G;
    else
        exception = MException('MyFunc:notValidSize', 'Array is not a valid size');
        throw(exception)
    end
        
end
