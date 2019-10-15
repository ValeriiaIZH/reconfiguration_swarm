function [ActiveDist]=homothetic_move(num, new_x_1_sd, new_y_1_sd, robot_coordinates, new_x_2, new_y_2)
%% перемещение по принципу гомотетии (2-3)
NumOfPoints_2 = size(new_x_2,1);
TargetKor = [new_x_1_sd; new_y_1_sd]';
if num >= NumOfPoints_2       
    ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
else
    ActiveRobotsUnsort = zeros(NumOfPoints_1, 2);
end
RobotKor = robot_coordinates;
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
% %% отображение таргентных точек и роботов перемещение гомотетия
% f3 = figure; 
% figure(f3);
% plot(x_c2, y_c2, 'r*', new_x_1_sd, new_y_1_sd, 'Om',...
%                        RobotKor(:,1), RobotKor(:,2), '.m',...
%                        new_x_2, new_y_2, '.k');
% hold on;
% plot(TargetAll(:,1),TargetAll(:,2), 'Og');
% grid on;
% for i = 1:size(robotCoordinates,1)      
%     text( new_x_1_sd(i), ...
%           new_y_1_sd(i), ...  
%           num2str(i));
%     text(x_1_sd(i),...
%          y_1_sd(i),...
%          num2str(i));  
% end
% отображение траекторий роботов 
plotTraj(ActiveRobots,PathKor);
Animation_2d(PathKor);
end

