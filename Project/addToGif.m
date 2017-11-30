function addToGif(i, frame, filename)
delay_time = 0.1;
%convert to right format
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
if i == 1 
  imwrite(imind,cm,filename,'gif', 'Loopcount',Inf,'DelayTime',delay_time); 
else 
  imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',delay_time); 
end 

end