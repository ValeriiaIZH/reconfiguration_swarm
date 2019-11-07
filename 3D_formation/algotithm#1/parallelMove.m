function [PathKor, PathTime, RobotCor0, ActiveDist0] = parallelMove(RobotCor, NewRobotCor, TargetNum)
global ROBOTNUM
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% создание массива активных роботов
RobotCor0 = RobotCor;
ActiveDist0 = zeros(size(ActiveRobotsUnsort,1), 1);                                                   
for i = 1:TargetNum                        
    TargetRast = zeros(1, size(RobotCor0,1));
    distdist = zeros(1, size(RobotCor0,1));
    % точки до каждого из
    for j = 1:TargetNum
            distdist(i,j)=pdist2(NewRobotCor(i,:), RobotCor0(j,:));
%             TargetRast(1, j)= pdist2(TargetCor(i,:), RobotCor2(j,:));                                                   % оставшихся роботов
    if i == j
            TargetRast(1, j) = distdist(i,j);
    end 
    end
    [DistValue, Ind] = max(TargetRast);
    ActiveDist0(i,1) =  DistValue;                  % значение расстояния 
    ActiveRobotsUnsort(i, :) = RobotCor0(Ind, :);  
%     RobotCor2(Ind, :) = [];
    if isempty(RobotCor0)
        break;
    end
end                                                
ActiveRobots = ActiveRobotsUnsort;
ActiveNewRobCor = NewRobotCor;
clear ActiveRobotsUnsort ;
[PathKor, PathTime] = animbycoordinates(ActiveDist0, ActiveRobots, ActiveNewRobCor);
end

