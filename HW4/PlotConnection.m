function PlotConnection(A_all, alpha1_range, alpha2_range)
    clf;
    var_names = {'$X$', '$Y$', '$\Theta$'};
    ax = zeros(3,1);
    for i = 1:3 % x, y, theta
        ax(i) = subplot(3,1,i);
        hold on
        x = [];
        y = []; 
        u = [];
        v = [];
        for a1 = 1:size(A_all,1)
            for a2 = 1:size(A_all,1)
                x = [x; alpha1_range(a1)];
                y = [y; alpha2_range(a2)];
                u = [u; A_all{a1, a2}(i,1)/100];
                v = [v; A_all{a1, a2}(i,2)/100];
            end
        end
        plot(x,y,'ro');
        quiver(x,y,u,v);
        xlabel('$\alpha_1$','interpreter','latex');
        ylabel('$\alpha_2$','interpreter','latex');
        title(var_names{i}, 'interpreter', 'latex');
        hold off;
    end
    allYLim = get(ax, {'YLim'});
    allYLim = cat(2, allYLim{:});
    set(ax,'YLim',[min(allYLim), max(allYLim)])
    
    allXLim = get(ax, {'XLim'});
    allXLim = cat(2, allXLim{:});
    set(ax,'XLim',[min(allXLim), max(allXLim)])
end