function [PathKor, PathTime] = calculate_rob_to_tar_v1(num, TargetAll, new_rob_cor)
%% расчет координат перемещение роботов к таргентным точкам
clear ActiveRobot TargetKor distdist TargetRast  DistValue
NumOfPoints_2 = size(TargetAll,1);
TargetKor = TargetAll;
if num >= NumOfPoints_2       
    ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
else
    ActiveRobotsUnsort = zeros(num, 2);
end
RobotKor = new_rob_cor;
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
for i = 1:size(TargetKor,1)
    TargetRast = zeros(1, size(RobotKor,1));      
    for j = 1:size(RobotKor,1)
         TargetRast(1, j)= pdist2(TargetKor(i,:), RobotKor(j,:)); 
    end 
    [DistValue, Ind] = min(TargetRast);
    ActiveDist(i,1) =  DistValue; % значение расстояния 
    ActiveRobotsUnsort(i, :) = RobotKor(Ind, :);  
    RobotKor(Ind, :) = [];
    if isempty(RobotKor)
        break;
    end
end
ActiveRobots = ActiveRobotsUnsort;
ActiveTargets = TargetKor;
% animation(Peak, Edge, [x_1_sd; y_1_sd]', TargetAll,...
%                           x_c2, y_c2);
title('Расчёт траекторий роботов');
hold on;
[PathKor, PathTime] = animbycoordinates(ActiveDist, ActiveRobots,...
                                                        ActiveTargets);
plotTraj_ANOTHER(ActiveRobots,PathKor);
Animation_2d(PathKor);
end


