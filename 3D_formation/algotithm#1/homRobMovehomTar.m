function [PathKor, PathTime] = homRobMovehomTar(homRobot,homTarget)
global ROBOTNUM
%% расчет координат перемещение роботов после гомотетии (3-4)
clear ActiveRobot ActiveRobots TargetKor ActiveRobotsUnsort TargetRast ActiveDist DistValue;
TargetKor = homTarget;
num = size(homTarget,1);
if ROBOTNUM >= num       
    ActiveRobotsUnsort = zeros(num, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
RobotKor = homRobot;
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
end


