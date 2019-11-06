function [PathKor, PathTime, ActiveDist] = homotheticMove(homRobot, ...
                                       NewRobotCor, homTarget)
global ROBOTNUM
%% перемещение по принципу гомотетии (2-3)
num = size(homTarget,1);
TargetKor = homRobot;
if ROBOTNUM >= num       
    ActiveRobotsUnsort = zeros(num, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
RobotKor = NewRobotCor;
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
for i = 1:size(TargetKor,1)
    TargetRast = zeros(1, size(RobotKor,1)); 
    distdist = zeros(1, size(RobotKor,1));
    for j = 1:size(RobotKor,1)
            distdist(i,j)=pdist2(TargetKor(i,:), RobotKor(j,:));
        if i == j
            TargetRast(1, j) = distdist(i,j);
        end
        
    end 
    [DistValue, Ind] = max(TargetRast);
    ActiveDist(i,1) =  DistValue;                  
    ActiveRobotsUnsort(i, :) = RobotKor(Ind, :);  
    if isempty(RobotKor)
        break;
    end
end
ActiveRobots = ActiveRobotsUnsort;
ActiveTargets = TargetKor;
[PathKor, PathTime] = animbycoordinates(ActiveDist, ActiveRobots,...
                                                        ActiveTargets);
end



