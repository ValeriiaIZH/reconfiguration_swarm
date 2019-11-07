function AnimatedMovement3(PathCor) 
% функция отображает перемещение роботов к таргетным точкам
% PathKor - массив, содержащий координаты роботов
global MASHROB 

for i = 1:size(PathCor,3)
h(i) = plot(NaN,NaN,'o',... 
                    'MarkerEdgeColor','b',...
                    'MarkerSize', MASHROB,...
                    'MarkerFaceColor','g');
end

for j = 1:size(PathCor,1)
    for i = 1:size(PathCor,3)
        set(h(i),'XData', PathCor(j,1,i), ...
                 'YData', PathCor(j,2,i), ...
                 'ZData', PathCor(j,3,i));      
    end
    pause(.1);
end


    


