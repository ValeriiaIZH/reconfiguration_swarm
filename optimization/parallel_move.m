function [ActiveDist] = parallel_move(TargetKor, num_of_robots, TargetAll, robot_coordinates, center)
%% расчёт траектории движения паралелльного переноса (1-2)
NumOfActiveRobot = num_of_robots;
NumOfPoints_2 = size(TargetAll,1); % количество таргентных точек

% определение числа активных роботов
if num_of_robots >= NumOfPoints_2       
    ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
else
    ActiveRobotsUnsort = zeros(num_of_robots, 2);
end
% создание массива активных роботов
RobotKor2 = robot_coordinates;
% массив расстояний от активного робота до его таргетной точки
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
for i = 1:size(TargetKor,1)
    % массив расстояний от точки до каждого из оставшихся роботов
    TargetRast = zeros(1, size(RobotKor2,1));  
    for j = 1:size(RobotKor2,1)
            distdist(i,j)=pdist2(TargetKor(i,:), RobotKor2(j,:));
        if i == j
            TargetRast(1, j) = distdist(i,j);
        end   
    end 
    [DistValue, Ind] = max(TargetRast);
    ActiveDist(i,1) =  DistValue;                  % значение расстояния 
    ActiveRobotsUnsort(i, :) = RobotKor2(Ind, :);  
    if isempty(RobotKor2)
        break;
    end
end
ActiveRobots = ActiveRobotsUnsort;
ActiveTargets = TargetKor;
% координаты перемещения роботов
[PathKor, ~] = animbycoordinates(ActiveDist, ActiveRobots, ...
                                                    ActiveTargets);
% отображение поверхности и роботов с центрами множеств
% animation(Peak, Edge, robot_coordinates, TargetAll, center);
title('Поверхность и роботы с центрами множеств(Параллельный перенос)');
% отображение новых координат роботов после параллельного переноса                      
hold on;
plot(TargetKor(:,1), TargetKor(:,2), 'om', center(:,1), center(:,2), '*r');
% построение траекторий паралелльного переноса и анимация переноса
plotTraj(ActiveRobots,PathKor);
% axis([-40 40 -40 40])
Animation_2d(PathKor)
end

