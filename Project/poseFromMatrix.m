function out = poseFromMatrix(group)
%     group = real(group); % for some reason it starts looking at imaginary values
	x = group(1,4);
	y = group(2,4);
	z = group(3,4);
    try
        gamma = atan2(group(3,2),group(3,3));
        alpha = atan2(group(2,1), group(1,1));
        % second term simplifies to cos(beta)
        beta = atan2(-group(3,1), group(1,1)*cos(alpha) + group(2,1)*sin(alpha));
    catch
        disp('Error!')
    end
%     try
%         if isvalid(group(3,1))
%             beta = asin(-group(3,1));
%         end
%     catch
%         blah = 1;
%     end
	out = [x, y, z, gamma, beta, alpha];
end