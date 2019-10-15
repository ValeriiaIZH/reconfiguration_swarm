function [PathKor, PathTime] = after_homothetic(new_x_2, new_y_2, num, new_x_1_sd, new_y_1_sd)
%% расчет координат перемещение роботов после гомотетии (3-4)
clear ActiveRobot ActiveRobots TargetKor ActiveRobotsUnsort TargetRast ActiveDist DistValue;
TargetKor = [new_x_2; new_y_2]';
NumOfPoints_2 = size(new_x_2,1);
if num >= NumOfPoints_2       
    ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
else
    ActiveRobotsUnsort = zeros(NumOfPoints_1, 2);
end
RobotKor = [new_x_1_sd; new_y_1_sd]';
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
for i = 1:size(TargetKor,1)
    TargetRast = zeros(1, size(RobotKor,1));      
    for j = 1:size(RobotKor,1)
         TargetRast(1, j)= pdist2(TargetKor(i,:), RobotKor(j,:)); 
    end 
    [DistValue, Ind] = min(TargetRast);
    ActiveDist(i,1) =  DistValue;                  % значение расстояния 
    ActiveRobotsUnsort(i, :) = RobotKor(Ind, :);  
    RobotKor(Ind, :) = [];
    if isempty(RobotKor)
        break;
    end
end
ActiveRobots = ActiveRobotsUnsort;
ActiveTargets = TargetKor;
[PathKor, PathTime] = animbycoordinates(ActiveDist, ActiveRobots,...
                                                        ActiveTargets);
plotTraj(ActiveRobots,PathKor);
Animation_2d(PathKor);
end

