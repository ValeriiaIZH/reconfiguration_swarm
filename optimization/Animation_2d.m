function Animation_2d(PathKor)
% функция анимации в 2D пути перемещения
global MASHROB;

for i = 1:size(PathKor,3)
h(i) = plot(NaN,NaN,'o',... 
                    'MarkerEdgeColor','m',...
                    'MarkerSize', MASHROB,...
                    'MarkerFaceColor','m');
end

for j = 1:size(PathKor,1)
    for i = 1:size(PathKor,3)
        set(h(i),'XData', PathKor(j,1,i), ...
                 'YData', PathKor(j,2,i));      
    end
    pause(.3);
end
delete(h);
end

