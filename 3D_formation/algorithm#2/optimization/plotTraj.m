function plotTraj(ActiveRobots,PathKor)
% функция отрисовки траектории движения
for i = 1:size(ActiveRobots,1)
        for j = 1:size(PathKor,1)
             obj = plot(PathKor(:, 1, i), ...
                   PathKor(:, 2, i),':b');
             
        end
   
end
delete(obj);
end

