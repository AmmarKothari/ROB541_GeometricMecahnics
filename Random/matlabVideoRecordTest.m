filename = 'testAnimated.gif';


f = figure(1);
clf(f);
ax = axes(f);
xlim([0,10]);
frame = [];
hold on
for i = 1:100
    % plot stuff
    start = i/100*pi;
    end_range = start + 2*pi;
    t = start:pi/16:end_range;
    x = cos(t);
    y = sin(t);
    if i ~= 0
        h1.delete()
        h2.delete()
    end
    h1 = plot(ax, t, x, 'r-');
    h2 = plot(ax, t, y, 'b-');
    drawnow
    
    % get frame
    frame = [frame, getframe(f)];
    
end

hold off