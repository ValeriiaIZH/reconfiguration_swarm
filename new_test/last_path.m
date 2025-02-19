function [PathKor, PathTime, ActiveRobots, ActiveDist] = last_path(num, TargetAll, new_x_2, new_y_2)
NumOfPoints_2 = size(TargetAll,1);
TargetKor = TargetAll;
    if num >= NumOfPoints_2       
        ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
    else
        ActiveRobotsUnsort = zeros(num, 2);
    end
    RobotKor = [new_x_2; new_y_2]';
    ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
    for i = 1:size(TargetKor,1)
        TargetRast = zeros(1, size(RobotKor,1));      
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
    plotTraj(ActiveRobots,PathKor)
    Animation_2d(PathKor)
end

