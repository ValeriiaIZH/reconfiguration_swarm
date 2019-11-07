function plotTraj(ActiveRobots,PathKor)
% функция отрисовки траектории движения
for i = 1:size(ActiveRobots,1)
        for j = 1:size(PathKor,1)
             plot3(PathKor(:, 1, i), ...
                  PathKor(:, 2, i), ...
                  PathKor(:, 3, i),':b');
        end
end
end