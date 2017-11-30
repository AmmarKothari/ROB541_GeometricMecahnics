function makeGif(frames, filename)
delay_time = 0.1;
for i=1:length(frames)
    %convert to right format
    im = frame2im(frames(i)); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if i == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',0,'DelayTime',delay_time); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',delay_time); 
    end 
end

end