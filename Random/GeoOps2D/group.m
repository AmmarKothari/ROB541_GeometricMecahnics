function out = group(g)
    if all(size(g) == [1,3])
        x = g(1);
        y = g(2);
        theta = g(3);
        out = [cos(theta), -sin(theta), x;
                sin(theta), cos(theta), y;
                0, 0, 1];
    elseif all(size(g) == [3,3])
        out = g;
    else
        exception = MException('MyFunc:notValidSize', 'Array is not a valid size');
        throw(exception)
    end
end