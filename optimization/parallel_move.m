function [ActiveDist] = parallel_move(TargetKor, num_of_robots, TargetAll, robot_coordinates, center)
%% ������ ���������� �������� ������������� �������� (1-2)
NumOfActiveRobot = num_of_robots;
NumOfPoints_2 = size(TargetAll,1); % ���������� ���������� �����

% ����������� ����� �������� �������
if num_of_robots >= NumOfPoints_2       
    ActiveRobotsUnsort = zeros(NumOfPoints_2, 2);
else
    ActiveRobotsUnsort = zeros(num_of_robots, 2);
end
% �������� ������� �������� �������
RobotKor2 = robot_coordinates;
% ������ ���������� �� ��������� ������ �� ��� ��������� �����
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);
for i = 1:size(TargetKor,1)
    % ������ ���������� �� ����� �� ������� �� ���������� �������
    TargetRast = zeros(1, size(RobotKor2,1));  
    for j = 1:size(RobotKor2,1)
            distdist(i,j)=pdist2(TargetKor(i,:), RobotKor2(j,:));
        if i == j
            TargetRast(1, j) = distdist(i,j);
        end   
    end 
    [DistValue, Ind] = max(TargetRast);
    ActiveDist(i,1) =  DistValue;                  % �������� ���������� 
    ActiveRobotsUnsort(i, :) = RobotKor2(Ind, :);  
    if isempty(RobotKor2)
        break;
    end
end
ActiveRobots = ActiveRobotsUnsort;
ActiveTargets = TargetKor;
% ���������� ����������� �������
[PathKor, ~] = animbycoordinates(ActiveDist, ActiveRobots, ...
                                                    ActiveTargets);
% ����������� ����������� � ������� � �������� ��������
% animation(Peak, Edge, robot_coordinates, TargetAll, center);
title('����������� � ������ � �������� ��������(������������ �������)');
% ����������� ����� ��������� ������� ����� ������������� ��������                      
hold on;
plot(TargetKor(:,1), TargetKor(:,2), 'om', center(:,1), center(:,2), '*r');
% ���������� ���������� ������������� �������� � �������� ��������
plotTraj(ActiveRobots,PathKor);
% axis([-40 40 -40 40])
Animation_2d(PathKor)
end

